/*
 *
 *  Created on: Jul, 2017
 *      Author: eneftci@uci.edu
 */
#include <time.h>
#include <libcaer/events/spike.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/imu6.h>
#include <libcaer/events/config.h>
#include "ext/uthash/utlist.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "main.h"
#include <libcaer/ringbuffer.h>
#include "modules/misc/in/input_common.h"
#include <sys/types.h>
#include <fcntl.h>
#include <limits.h>
#include <ncurses.h>

struct HWFilter_state {
	// user settings
	//bool setCam;
	//bool loadBiases;
	//bool setSram;
	bool init;
	caerRingBuffer dataTransfer;
	// usb utils
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
int labelon = 1024+1034;
int labeloffset = 1024*2;
uint tail = 0;
int32_t addr;
int32_t diff;
char buffer [1000000];
int nimu = 0;
float mx=0; float my=0; float mz=0;

bool learn_enable = true;
float m2x=0, deltax=0, deltax2=0;
float m2y=0, deltay=0, deltay2=0;
float m2z=0, deltaz=0, deltaz2=0;
float varaccel  = 0;
//
//
typedef struct HWFilter_state *HWFilterState;

static bool caerNSATInterfaceModuleInit(caerModuleData moduleData);
static void caerNSATInterfaceModuleRun(caerModuleData moduleData, caerEventPacketContainer in, caerEventPacketContainer *out);
static void caerNSATInterfaceModuleExit(caerModuleData moduleData);
static void caerNSATInterfaceModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID);
caerSpikeEventPacket read_nsat_events(int fd, caerModuleData moduleData);

caerSpikeEventPacket read_nsat_events(int fd, caerModuleData moduleData) {
    int i, time;
    int num_events = 0;
    int n;
    int evcounter = 0;
    int buffer;
    int relbuffer;
    caerSpikeEventPacket nsatPacket = caerSpikeEventPacketAllocate(1000000,moduleData->moduleID,0);
    caerEventPacketHeaderSetEventTSOffset(nsatPacket, 4);
    caerEventPacketHeaderSetEventType(nsatPacket, SPIKE_EVENT);
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
            int core = 0;
            int chipid = 0;
            int neuronid = 0;
            if (3172<=buffer && buffer<3202){
              neuronid = buffer-3172;
              core = 3;
              chipid = 3;
            } else if (0<=buffer && buffer<2048){
              neuronid = buffer%256;
              core = (buffer/256)%4;
              chipid = (buffer/1024);
            } else if (2048<=buffer && buffer<3072){
              relbuffer = (buffer - 2048)%256;
              neuronid = (relbuffer%64)%8 +16*((relbuffer%64)/8)+ ((relbuffer/64)/2)*128 + ((relbuffer/64)%2)*8 ;
              core = (buffer/256)%4;
              chipid = (buffer/1024);
            } else if (3072<=buffer && buffer<3172){
              relbuffer = buffer - 3072;
              neuronid = relbuffer%10 + 16*(relbuffer/10);
              core = 2;
              chipid = 3;
            } 
            caerSpikeEvent tmpevent = caerSpikeEventPacketGetEvent(nsatPacket, evcounter+i);
            caerSpikeEventSetTimestamp(tmpevent, 1000*(time-1));
            caerSpikeEventSetNeuronID(tmpevent, neuronid);
            caerSpikeEventSetSourceCoreID(tmpevent, core);
            caerSpikeEventSetChipID(tmpevent, chipid);
            caerSpikeEventValidate(tmpevent, nsatPacket);
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
    { .type = IMU6_EVENT, .number = 1, .readOnly = true },
    { .type = CONFIG_EVENT, .number = 1, .readOnly = true },
};

static const struct caer_event_stream_out moduleOutputs[] = { { .type = POLARITY_EVENT}, { .type = SPIKE_EVENT} };


static const struct caer_module_info moduleInfo = {
	.version = 1, .name = "nsatinterface",
	.description = "DAVIS NSAT Interface",
	.type = CAER_MODULE_PROCESSOR,
	.memSize = sizeof(struct HWFilter_state),
	.functions = &caerNSATInterfaceModuleFunctions,
	.inputStreams = moduleInputs,
	.inputStreamsSize = 3,
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
		sshsNodeCreateShort(sourceInfoNode, "dataSizeX", 64, 64, 64, SSHS_FLAGS_NORMAL, "number of neurons in X");
		sshsNodeCreateShort(sourceInfoNode, "dataSizeY", 64, 64, 64, SSHS_FLAGS_NORMAL, "number of neurons in Y");
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

  caerLog(CAER_LOG_NOTICE, __func__, "Opening NSAT events file, %s", sshsNodeGetString(moduleData->moduleNode, "nsat_filename"));
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
    caerIMU6EventPacket imu6 =
		(caerIMU6EventPacket) caerEventPacketContainerFindEventPacketByTypeConst(in, IMU6_EVENT);
   

    //if (imu6 != NULL){
    //  CAER_IMU6_ITERATOR_VALID_START(imu6)
    //      float imux; float imuy; float imuz;
    //      nimu += 1;
    //      imux = caerIMU6EventGetAccelX(caerIMU6IteratorElement);
    //      imuy = caerIMU6EventGetAccelY(caerIMU6IteratorElement);
    //      imuz = caerIMU6EventGetAccelZ(caerIMU6IteratorElement);

    //      mx = fabs(imux)*.01 + mx*.975;
    //      my = fabs(imuy)*.01 + my*.975;
    //      mz = fabs(imuz)*.01 + mz*.975;

    //  CAER_IMU6_ITERATOR_VALID_END
    //  varaccel = (mx + my + mz) -.52;
    //  if (learn_enable){
    //  if(varaccel>.05){
    //    c = 1+labeloffset;
    //    label_counter=1000; 
    //  } 
    //  if(varaccel>.1){
    //    c = 2+labeloffset;
    //    label_counter=1000; 
    //  } 
    //  if(varaccel>.2){
    //    c = 3+labeloffset;
    //    label_counter=100; 
    //  }
    //   if(varaccel>.05) printf("%d c\n", c-labeloffset);
    //  }
    //}
    //
    if (config!=NULL){
        caerConfigurationEvent tmpevent = caerConfigurationEventPacketGetEvent(config, 0);
        label = (int)caerConfigurationEventGetParameter(tmpevent);
    //if (label == 59) learn_enable = false;
    //if (label == 60) learn_enable = true;
        if (label>=0 && label<10){
          c = label+labeloffset;
          label_counter+=1000; 
          label = -100;
        }
    }

    if (polarity==NULL){ return; }








    int32_t numEvents= caerEventPacketHeaderGetEventNumber(polarity);

    //-----------nsat packet
    caerPolarityEventPacket vizPacket = caerPolarityEventPacketAllocate(numEvents,moduleData->moduleID,0);;
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

            if(label_counter>0 && ((intertick%1==0))){
                if(label_counter>0){
                counter+=1;                             
                memcpy(buffer+tail,&ocp,cp);
                nloc=tail;
                tail+=cp;
                if(intertick%40==0){
                memcpy(buffer+nloc,&tcp,cp);
                counter+=1;                             
                memcpy(buffer+tail,&zcp,cp);
                printf("label: %d \n",c);
                memcpy(buffer+tail+cp,&c,cp);
                tail+=2*cp;
                }
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
        if (x>=90 && x<150 && y>=60 && y<120)
        {
            addr=((y-60)/2)*32+(x-90)/2+1024*pol;                        
            memcpy(buffer+tail,&zcp,cp);
            memcpy(buffer+tail+cp,&addr,cp);
            tail+=2*cp;
            counter+=1;

            caerPolarityEvent tmpevent = caerPolarityEventPacketGetEvent(vizPacket, j);
            caerPolarityEventSetTimestamp(tmpevent,ts);
            caerPolarityEventSetX(tmpevent,x-90);
            caerPolarityEventSetY(tmpevent,y-60);
            caerPolarityEventSetPolarity(tmpevent,pol);  
            caerPolarityEventValidate(tmpevent, vizPacket);
            vizcounter+=1;
        }

    }
    caerSpikeEventPacket nsatPacket = read_nsat_events(state->nsat_fp, moduleData); 
    evcounter = caerEventPacketHeaderGetEventNumber(nsatPacket);
    caerEventPacketHeaderSetEventNumber(vizPacket, vizcounter);
    caerEventPacketHeaderSetEventValid(vizPacket, vizcounter);

    *out = caerEventPacketContainerAllocate(2);
    caerEventPacketContainerSetEventPacket(*out, 1, (caerEventPacketHeader) vizPacket);

    if (nsatPacket == NULL) {
      return; // Error.
    } else {
//
//    // Everything that is in the out packet container will be automatically freed after main loop.
      // Add output packet to packet container.
        caerEventPacketContainerSetEventPacket(*out, 0, (caerEventPacketHeader) nsatPacket);
        //printf("NSAT events: %d \n", caerEventPacketHeaderGetEventValid(nsatPacket));
        //CAER_SPIKE_ITERATOR_VALID_START(nsatPacket)
        //    printf("ts %d   ", caerSpikeEventGetTimestamp(caerSpikeIteratorElement));
        //    printf("neur %d   ", caerSpikeEventGetNeuronID(caerSpikeIteratorElement));
        //    printf("core %d   ", caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement));
        //    printf("chip %d   \n", caerSpikeEventGetChipID(caerSpikeIteratorElement));
        //CAER_SPIKE_ITERATOR_VALID_END
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


