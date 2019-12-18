/**
 *  Exploration example definitions.
 */

#ifndef __EXPLORATION_H__
#define __EXPLORATION_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3usb.h>
#include <cyu3i2c.h>
#include <cyu3spi.h>
#include <cyu3gpio.h>
#include <cyu3uart.h>
#include <cyu3error.h>
#include <cyu3types.h>
#include <cyu3system.h>
#include <cyu3usbconst.h>

#define APPTHREAD_STACK       (0x0800)    /* App thread stack size */
#define APPTHREAD_PRIORITY    (8)         /* App thread priority */

/* Give a timeout value of 5s for any flash programming. */
#define CY_FX_FLASH_PROG_TIMEOUT                (5000)

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];

#endif /* _INCLUDED_CYFXFLASHPROG_H_ */

/*[]*/
