#ifndef SERVICES_H_
#define SERVICES_H_

#include <random>

#include "servicesBase.h"
#include "utils/patterns/singleton.h"

// forward declarations for services, to avoid having to
// make this a huge include-fest
class OttoBoard;

// events we provide
DECLARE_EVENT_ID(EVT_MOTION_MODULE_DATA);
DECLARE_EVENT_ID(EVT_STEREOIMAGE_RECEIVED);
DECLARE_EVENT_ID(EVT_STEREOIMAGEDEPTHS_RECEIVED);
DECLARE_EVENT_ID(EVT_STEREOIMAGES_CAPTURED)

/**
 ** The Services class is the one-point-stop-shop for all
 ** things service-related. Or in other words - put all the
 ** crap here that needs to be initialized once and be
 ** available more or less globally.
 **
 */

class Services
	: public ServicesBase
	, public Singleton<Services>
{
	friend class Singleton<Services>;

private:
	Services();

public:
	virtual ~Services();
	virtual bool init(int argc, char* argv[], ServiceInitStructure* sis = nullptr) override;

	inline OttoBoard& getOttoBoard() { return *ottoBoard; }
	inline std::default_random_engine& getRandomEngine() { return randomEngine; }

protected:
	OttoBoard     *ottoBoard;
	std::default_random_engine randomEngine;
};

extern Services &services;

#endif /* SERVICES_H_ */
