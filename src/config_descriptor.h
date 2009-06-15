#ifndef _CONFIGDESCRIPTOR_H_
#define _CONFIGDESCRIPTOR_H_

#include "LUFA/Drivers/USB/USB.h"
#include "garmin_device.h"

#define MAX_CONFIG_DESCRIPTOR_SIZE 512

enum GarminHost_GetConfigDescriptorDataCodes_t
{
    SuccessfulConfigRead,       // config descriptor was processed succesfully
    ControlError,               // control request to the device failed
    DescriptorTooLarge,         // device's config descriptor is too large to process
    InvalidConfigDataReturned,  // device returned invalid config descriptor
    NoInterfaceFound,           // a Garmin interface wasn't found
    NoEndpointFound             // endpoints did not match a Garmin device
};
	
uint8_t ProcessConfigurationDescriptor(void);
uint8_t DComp_NextGarminInterface(void* CurrentDescriptor);
uint8_t DComp_NextGarminInterfaceDataEndpoint(void* CurrentDescriptor);

#endif // _CONFIGDESCRIPTOR_H_
