/*
 ## Cypress USB 3.0 Platform header file (cyfxflashprog.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file contains the externants used by the flash programmer application. */

#ifndef _INCLUDED_CYFXFLASHPROG_H_
#define _INCLUDED_CYFXFLASHPROG_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define APPTHREAD_STACK       (0x0800)    /* App thread stack size */
#define APPTHREAD_PRIORITY    (8)         /* App thread priority */

/* Give a timeout value of 5s for any flash programming. */
#define CY_FX_FLASH_PROG_TIMEOUT                (5000)


/* USB vendor requests supported by the application. */

/* USB vendor request to read the 8 byte firmware ID. This will return content 
 * of glFirmwareID array. */
#define CY_FX_RQT_ID_CHECK                      (0xB0)

/* USB vendor request to write to I2C EEPROM connected. The EEPROM page size is
 * fixed to 64 bytes. The I2C EEPROM address is provided in the value field. The
 * memory address to start writing is provided in the index field of the request.
 * The maximum allowed request length is 4KB. */
#define CY_FX_RQT_I2C_EEPROM_WRITE              (0xBA)

/* USB vendor request to read from I2C EEPROM connected. The EEPROM page size is
 * fixed to 64 bytes. The I2C EEPROM address is provided in the value field. The
 * memory address to start reading from is provided in the index field of the
 * request. The maximum allowed request length is 4KB. */
#define CY_FX_RQT_I2C_EEPROM_READ               (0xBB)

/* USB vendor request to read data from SYS_MEM starting at 32-bit aligned given
 * address. The MS 16-bit of start address is provided in the index field and the
 * LS 16-bit is provided in the value field of the request. */
#define CY_FX_RQT_SYS_MEM_READ                  (0xC0)

/* USB vendor request to write data to SPI flash connected. The flash page size is
 * fixed to 256 bytes. The memory address to start the write is provided in the
 * index field of the request. The maximum allowed request length is 4KB. */
#define CY_FX_RQT_SPI_FLASH_WRITE               (0xC2)

/* USB vendor request to read data from SPI flash connected. The flash page size is
 * fixed to 256 bytes. The memory address to start the read is provided in the index
 * field of the request. The maximum allowed request length is 4KB. */
#define CY_FX_RQT_SPI_FLASH_READ                (0xC3)

/* USB vendor request to erase a sector on SPI flash connected. The flash sector
 * size is fixed to 64KB. The sector address is provided in the index field of
 * the request. The erase is carried out if the value field is non-zero. If this 
 * is zero, then the request returns the write in progress (WIP) bit. WIP should
 * be 0 before issuing any further transactions. */
#define CY_FX_RQT_SPI_FLASH_ERASE_POLL          (0xC4)

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

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXFLASHPROG_H_ */

/*[]*/
