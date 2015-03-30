#ifndef _OPENNIDEVICELISTENER_H_
#define _OPENNIDEVICELISTENER_H_

#include <OpenNI.h>

using namespace openni;

class OpenNIDeviceListener : public OpenNI::DeviceConnectedListener,
									public OpenNI::DeviceDisconnectedListener,
									public OpenNI::DeviceStateChangedListener
{
public:
	virtual void onDeviceStateChanged(const DeviceInfo* pInfo, DeviceState state) 
	{
		printf("Device \"%s\" error state changed to %d\n", pInfo->getUri(), state);
	}

	virtual void onDeviceConnected(const DeviceInfo* pInfo)
	{
		printf("Device \"%s\" connected\n", pInfo->getUri());
	}

	virtual void onDeviceDisconnected(const DeviceInfo* pInfo)
	{
		printf("Device \"%s\" disconnected\n", pInfo->getUri());
	}
};

#endif
