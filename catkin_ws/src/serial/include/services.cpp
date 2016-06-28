#include "services.h"
#include "modules/moduleDataExchangeEvent.h"

#include <chrono>

#include "platform/hardware/otto/ottoBoard.h"

// define provided events
DEFINE_EVENT_ID(EVT_MOTION_MODULE_DATA, "data from motion module");
DEFINE_EVENT_ID(EVT_STEREOIMAGES_CAPTURED, "stereo camera captured new image and depths image");

Services &services = Services::getInstance();

/**
 ** Constructor, get references to singletons and initialize
 ** everything else to NULL
 */
Services::Services() :
	randomEngine(
		std::default_random_engine(
			std::chrono::system_clock::now().time_since_epoch().count()
		)
	)
{
	ottoBoard = new OttoBoard();
}

Services::~Services() {
	delete ottoBoard;
	ottoBoard = NULL;
}

bool Services::init(int argc, char* argv[], ServiceInitStructure* sis) {
	if (false == ServicesBase::init(argc, argv, sis))
		return false;

	// register events
	REGISTER_EVENT_ID( EVT_MOTION_MODULE_DATA );
	REGISTER_EVENT_ID( EVT_STEREOIMAGES_CAPTURED );

	setupModuleDataExchange(); // TODO: make generic

	ottoBoard->init();
	return true;
}
