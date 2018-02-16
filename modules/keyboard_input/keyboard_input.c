/*
 *
 *  Created on: Jul, 2017
 *      Author: eneftci@uci.edu
 */
#include <time.h>
#include "ext/uthash/utlist.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <ncurses.h>
#include <libcaer/events/config.h>

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

struct HWFilter_state {
	bool init;
	sshsNode eventSourceConfigNode;
	int16_t sourceID;
};

//
//
typedef struct HWFilter_state *HWFilterState;

static bool caerKeybInModuleInit(caerModuleData moduleData);
static void caerKeybInModuleRun(caerModuleData moduleData, caerEventPacketContainer in, caerEventPacketContainer *out);
static void caerKeybInModuleExit(caerModuleData moduleData);
static void caerKeybInModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerKeybInModuleFunctions = { 
  .moduleInit =	&caerKeybInModuleInit,
  .moduleRun = &caerKeybInModuleRun,
  .moduleExit = &caerKeybInModuleExit,
  .moduleReset =&caerKeybInModuleReset };

static const struct caer_event_stream_out moduleOutputs[] = {
    { .type = CONFIG_EVENT}
};

static const struct caer_module_info moduleInfo = {
	.version = 1, .name = "spk2dyn",
	.description = "Dynapse Configuration from External Events",
	.type = CAER_MODULE_INPUT,
	.memSize = sizeof(struct HWFilter_state),
	.functions = &caerKeybInModuleFunctions,
	.outputStreams = moduleOutputs,
	.outputStreamsSize = CAER_EVENT_STREAM_OUT_SIZE(moduleOutputs),
	.inputStreams = NULL,
	.inputStreamsSize = NULL
};

caerModuleInfo caerModuleGetInfo(void) {
    return (&moduleInfo);
}

//s

static bool caerKeybInModuleInit(caerModuleData moduleData) {
	HWFilterState state = moduleData->moduleState;


	sshsNodeCreateString(moduleData->moduleNode, "current_keypress", "a", 0, 1, SSHS_FLAGS_NORMAL, "Current Keypress");

	// Wait for input to be ready. All inputs, once they are up and running, will
	// have a valid sourceInfo node to query, especially if dealing with data.

  //cbreak();
  //initscr();
  //noecho();
  //nodelay(stdscr, true);

  caerLog(CAER_LOG_NOTICE, __func__, "Keyboard module init completed");
	return (true);
}

static void caerKeybInModuleRun(caerModuleData moduleData, caerEventPacketContainer in, caerEventPacketContainer *out) {
UNUSED_ARGUMENT(in);
char * s = sshsNodeGetString(caerMainloopGetSourceNode(16), "current_keypress");
if(s[0] == 120) return;
*out = caerEventPacketContainerAllocate(1);
caerConfigurationEventPacket keypressPacket = caerConfigurationEventPacketAllocate(1,moduleData->moduleID,0);

//UGLY TELLURIDE QUICK FIX
caerConfigurationEvent tmpevent = caerConfigurationEventPacketGetEvent(keypressPacket, 0);
caerConfigurationEventSetTimestamp(tmpevent,0);
// caerConfigurationEventSetParameter(tmpevent,(int)(s[0])-48);
caerConfigurationEventSetParameter(tmpevent, atoi(s));
caerConfigurationEventSetParameterAddress(tmpevent,0);
caerConfigurationEventSetModuleAddress(tmpevent,0);
caerEventPacketHeaderSetEventValid(keypressPacket, 1);
caerEventPacketContainerSetEventPacket(*out, 0, (caerEventPacketHeader) keypressPacket);
sshsNodePutString(caerMainloopGetSourceNode(16), "current_keypress", "x");
}

static void caerKeybInModuleExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

}

static void caerKeybInModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

}


