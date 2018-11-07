#ifndef  __VERSION_H__
#define  __VERSION_H__


#define  FIRMWARE_MAJOR_VERSION                 1
#define  FIRMWARE_MINOR_VERSION                 1

#define  MAKE_FIRMWARE_VERSION(x,y)             ((x)<<8|(y))
#define  FIRMWARE_VERSION                       MAKE_FIRMWARE_VERSION(FIRMWARE_MAJOR_VERSION,FIRMWARE_MINOR_VERSION)












#endif