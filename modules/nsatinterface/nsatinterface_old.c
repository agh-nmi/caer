/*
 *
 *  Created on: Jul, 2017
 *      Author: eneftci@uci.edu
 */
#include <time.h>
#include <libcaer/events/spike.h>
#include <libcaer/events/config.h>
#include "ext/uthash/utlist.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "main.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "modules/misc/in/input_common.h"
#include <sys/types.h>
#include <fcntl.h>
#include <limits.h>
#include <ncurses.h>

int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

int flush(){
  int c;
  
  while(kbhit()){
    getch();
  }
  return c;
}

struct HWFilter_state {
	// user settings
	//bool setCam;
	//bool loadBiases;
	//bool setSram;
	bool init;
	RingBuffer dataTransfer;
	// usb utils
	caerDeviceHandle deviceState;
	sshsNode eventSourceConfigNode;
  FILE * davis_fp;
  int nsat_fp;
	int16_t sourceID;
};

size_t s8 = sizeof(int);
int cp = sizeof(int);
int zcp = 0;
int ocp = 1;
int tcp = 2;
int counter = 0;
int evcounter = 0;
int vizcounter = 0;
int intertick = 0;
int tss = 0 ;
int nloc = 0;
int label, c;
int z;
uint label_counter = 0;
int labelon = 794;
uint tail = 0;
int32_t addr;
int32_t diff;
char buffer [1000000];

//
//
typedef struct HWFilter_state *HWFilterState;

static bool caerNSATInterfaceModuleInit(caerModuleData moduleData);
static void caerNSATInterfaceModuleRun(caerModuleData moduleData, caerEventPacketContainer in, caerEventPacketContainer *out);
static void caerNSATInterfaceModuleExit(caerModuleData moduleData);
static void caerNSATInterfaceModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID);
caerPolarityEventPacket read_nsat_events(int fd, caerModuleData moduleData);

caerPolarityEventPacket read_nsat_events(int fd, caerModuleData moduleData) {
    int i, time;
    int num_events = 0;
    int n;
    int evcounter = 0;
    int buffer;
    caerPolarityEventPacket nsatPacket = caerPolarityEventPacketAllocate(100000,moduleData->moduleID,0);
    caerEventPacketHeaderSetEventTSOffset(nsatPacket, 4);
    caerEventPacketHeaderSetEventType(nsatPacket, POLARITY_EVENT);
    caerEventPacketHeaderSetEventNumber( nsatPacket, 0);
    caerEventPacketHeaderSetEventValid( nsatPacket, 0);

    for(;;){
    n = read(fd, &buffer, s8);
    if (n > 0) { //Read all pending events
        while (n != 4) { lseek(fd, -n, SEEK_CUR); n = read(fd, &buffer, s8); }
        time = buffer;
        n = read(fd, &buffer, s8);
        while (n != 4) { lseek(fd, -n, SEEK_CUR); n = read(fd, &buffer, s8); }
        num_events = buffer;

        for (i = 0; i < num_events; ++i) {
            n = read(fd, &buffer, s8);
            while (n != 4) { lseek(fd, -n, SEEK_CUR); n = read(fd, &buffer, s8); }
            //do stuff
            caerPolarityEvent tmpevent = caerPolarityEventPacketGetEvent(nsatPacket, evcounter+i);
            if(2156<=buffer<2256){
            buffer -= 2156;
            caerPolarityEventSetTimestamp(tmpevent, 1000*(time-1));
            caerPolarityEventSetX(tmpevent,buffer/10);
            caerPolarityEventSetY(tmpevent,buffer%10);
            caerPolarityEventSetPolarity(tmpevent,1);  
            caerPolarityEventValidate(tmpevent, nsatPacket);
            } else if (2256<=buffer<2287){
            buffer -= 2156;
            caerPolarityEventSetTimestamp(tmpevent, 1000*(time-1));
            caerPolarityEventSetX(tmpevent,buffer/10);
            caerPolarityEventSetY(tmpevent,buffer%10);
            caerPolarityEventSetPolarity(tmpevent,0);  
            caerPolarityEventValidate(tmpevent, nsatPacket);
            }

        }
        evcounter += num_events;
        caerEventPacketHeaderSetEventNumber( nsatPacket, evcounter);
        caerEventPacketHeaderSetEventValid( nsatPacket, evcounter);
    } else {
      //caerPolarityEvent tmpevent = caerPolarityEventPacketGetEvent(nsatPacket, 0);
      //caerPolarityEventSetTimestamp(tmpevent,1000*(time-1));
      //caerPolarityEventSetX(tmpevent,-1);
      //caerPolarityEventSetY(tmpevent,-1);
      //caerPolarityEventSetPolarity(tmpevent,0);  
      //caerPolarityEventValidate(tmpevent, nsatPacket);

      break;
    }
    }

    return nsatPacket;
}

static struct caer_module_functions caerNSATInterfaceModuleFunctions = { 
  .moduleInit =	&caerNSATInterfaceModuleInit,
  .moduleRun = &caerNSATInterfaceModuleRun,
  .moduleExit = &caerNSATInterfaceModuleExit,
  .moduleReset =&caerNSATInterfaceModuleReset };

static const struct caer_event_stream_in moduleInputs[] = {
    { .type = POLARITY_EVENT, .number = 1, .readOnly = true },
    { .type = CONFIG_EVENT, .number = 1, .readOnly = true }
};

static const struct caer_event_stream_out moduleOutputs[] = { { .type = SPIKE_EVENT} };


static const struct caer_module_info moduleInfo = {
	.version = 1, .name = "nsatinterface",
	.description = "DAVIS NSAT Interface",
	.type = CAER_MODULE_PROCESSOR,
	.memSize = sizeof(struct HWFilter_state),
	.functions = &caerNSATInterfaceModuleFunctions,
	.inputStreams = moduleInputs,
	.inputStreamsSize = CAER_EVENT_STREAM_IN_SIZE(moduleInputs),
	.outputStreams = moduleOutputs,
	.outputStreamsSize = CAER_EVENT_STREAM_OUT_SIZE(moduleOutputs),
};

caerModuleInfo caerModuleGetInfo(void) {
    return (&moduleInfo);
}

//s

static bool caerNSATInterfaceModuleInit(caerModuleData moduleData) {
	HWFilterState state = moduleData->moduleState;

	// Wait for input to be ready. All inputs, once they are up and running, will
	// have a valid sourceInfo node to query, especially if dealing with data.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodeCreateShort(sourceInfoNode, "dataSizeX", 10, 10, 10*10, SSHS_FLAGS_NORMAL, "number of neurons in X");
		sshsNodeCreateShort(sourceInfoNode, "dataSizeY", 10, 10, 10*10, SSHS_FLAGS_NORMAL, "number of neurons in Y");
	}
	int16_t *inputs = caerMainloopGetModuleInputIDs(moduleData->moduleID, NULL);
	if (inputs == NULL) {
		return (false);
	}

	state->sourceID = inputs[0];
	free(inputs);

	sshsNodeCreateString(moduleData->moduleNode, "davis_filename", "/tmp/dvs", 0, 19, SSHS_FLAGS_NORMAL, "DAVIS output filename");
	sshsNodeCreateString(moduleData->moduleNode, "nsat_filename", "/tmp/nsat", 0, 19, SSHS_FLAGS_NORMAL, "NSAT output filename");


	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->sourceID));

  state->davis_fp = fopen(sshsNodeGetString(moduleData->moduleNode, "davis_filename"), "a");
  state->nsat_fp = open(sshsNodeGetString(moduleData->moduleNode, "nsat_filename"), O_RDONLY);

  caerLog(CAER_LOG_NOTICE, __func__, "NSAT module init completed");

	return (true);


}

static void caerNSATInterfaceModuleRun(caerModuleData moduleData, caerEventPacketContainer in, caerEventPacketContainer *out) {
    HWFilterState state = moduleData->moduleState;
    caerPolarityEventPacket polarity =
		(caerPolarityEventPacket) caerEventPacketContainerFindEventPacketByTypeConst(in, POLARITY_EVENT);
    caerConfigurationEventPacket config =
		(caerConfigurationEventPacket) caerEventPacketContainerFindEventPacketByTypeConst(in, CONFIG_EVENT);

    
    if (config!=NULL){
    caerConfigurationEvent tmpevent = caerConfigurationEventPacketGetEvent(config, 0);
    label = (int)caerConfigurationEventGetParameter(tmpevent);
    printf("label %d\n", label);
    if (label>=0 && label<10){
      c = label+784;
      label_counter+=50; 
      label = -100;
    }
    }

    if (polarity==NULL){ return; }








    int32_t numEvents= caerEventPacketHeaderGetEventNumber(polarity);

    //-----------nsat packet
    caerPolarityEventPacket vizPacket = caerPolarityEventPacketAllocate(numEvents,1,0);;
    vizcounter = 0;
    caerEventPacketHeaderSetEventNumber( vizPacket, vizcounter);
    caerEventPacketHeaderSetEventValid(  vizPacket, vizcounter);
    for (int32_t j=0; j<numEvents; j++)
    {
        caerPolarityEvent ev = caerPolarityEventPacketGetEvent(polarity, j);
        int32_t ts = caerPolarityEventGetTimestamp(ev)/1000;
        uint16_t x = caerPolarityEventGetX(ev);
        uint16_t y = caerPolarityEventGetY(ev);
        uint16_t pol = caerPolarityEventGetPolarity(ev);

        if(ts>tss){
          //tick!
          diff=ts-tss;
          memcpy(buffer+nloc,&counter,cp);
          fwrite(buffer, cp, tail/cp, state->davis_fp);
          //fflush(fp);
          tail=0;
          nloc=0;
          for(z=1; z<=diff; z++){
            counter = 0;
            intertick=tss+z;
            memcpy(buffer+tail,&intertick,cp);
            tail+=cp;

            if(label_counter>0 && ((intertick%4==0))){
                if(label_counter>0){
                printf("label: %d\n", label_counter);
                counter+=2;                             
                memcpy(buffer+tail,&tcp,cp);
                nloc=tail;
                tail+=cp;
                printf("c label: %d\n", c);
                memcpy(buffer+tail,&zcp,cp);
                memcpy(buffer+tail+cp,&c,cp);
                tail+=2*cp;
                memcpy(buffer+tail,&zcp,cp);
                memcpy(buffer+tail+cp,&labelon,cp);
                tail+=2*cp;
                label_counter-=1;
                } 

            } else {
            memcpy(buffer+tail,&zcp,cp);
            nloc=tail;
            tail+=cp;
          }
          }
          tss = ts;
        }
        if (x>=92 && x<148 && y>=62 && y<118)
        {
            addr=((y-62)/2)*28+(x-92)/2;                        
            memcpy(buffer+tail,&zcp,cp);
            memcpy(buffer+tail+cp,&addr,cp);
            tail+=2*cp;
            counter+=1;

            caerPolarityEvent tmpevent = caerPolarityEventPacketGetEvent(vizPacket, j);
            caerPolarityEventSetTimestamp(tmpevent,ts);
            caerPolarityEventSetX(tmpevent,x-92);
            caerPolarityEventSetY(tmpevent,y-62);
            caerPolarityEventSetPolarity(tmpevent,pol);  
            caerPolarityEventValidate(tmpevent, vizPacket);
            vizcounter+=1;
        }

    }
    caerPolarityEventPacket nsatPacket = read_nsat_events(state->nsat_fp, moduleData); 
    evcounter = caerEventPacketHeaderGetEventNumber(nsatPacket);
    caerEventPacketHeaderSetEventNumber(vizPacket, vizcounter);
    caerEventPacketHeaderSetEventValid(vizPacket, vizcounter);
    free(vizPacket);

    *out = caerEventPacketContainerAllocate(1);
    if (nsatPacket == NULL) {
      return; // Error.
    } else {
//
//    // Everything that is in the out packet container will be automatically freed after main loop.
      // Add output packet to packet container.
        caerEventPacketContainerSetEventPacket(*out, 0, (caerEventPacketHeader) nsatPacket);
        //printf("NSAT events: %d \n", caerEventPacketHeaderGetEventValid(nsatPacket));
        //CAER_POLARITY_ITERATOR_VALID_START(nsatPacket)
        //    printf("%d   ", caerPolarityEventGetTimestamp(caerPolarityIteratorElement));
        //    printf("%d   ", caerPolarityEventGetX(caerPolarityIteratorElement));
        //    printf("%d   ", caerPolarityEventGetY(caerPolarityIteratorElement));
        //    printf("%d \n", caerPolarityEventGetPolarity(caerPolarityIteratorElement));  
        //CAER_POLARITY_ITERATOR_VALID_END
    }

}

static void caerNSATInterfaceModuleExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	HWFilterState state = moduleData->moduleState;
  fclose(state->davis_fp);
  close(state->nsat_fp);

}

static void caerNSATInterfaceModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);
	UNUSED_ARGUMENT(moduleData);

}


