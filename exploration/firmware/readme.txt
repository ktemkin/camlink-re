
                        CYPRESS SEMICONDUCTOR CORPORATION
                                    FX3 SDK

USB Flash Programmer Example
----------------------------

  This example illustrates the use of the FX3 firmware APIs to implement
  an application to program firmware onto I2C/SPI EEPROM devices.

  The device enumerates as a vendor specific USB device with only the control
  endpoint and provides a set of vendor commands to read/write the data on
  I2C EEPROM devices and SPI flash devices. The SPI command protocol used
  is based on the Micron M25PXX devices.

  A SPI flash write requires the following sequences:

  1. Erase the required sector: CY_FX_RQT_SPI_FLASH_ERASE_POLL with wValue
     as 1 and length as 0.
  2. Wait for erase to complete: CY_FX_RQT_SPI_FLASH_ERASE_POLL with wValue
     as 0 and IN length as 1. Repeat this until the returned value has WIP zero.
  3. Write data to page: CY_FX_RQT_SPI_FLASH_WRITE with correct parameters.
  4. Wait for write to complete: CY_FX_RQT_SPI_FLASH_ERASE_POLL with wValue
     as 0 and IN length as 1. Repeat this until the returned value has WIP zero.

  Files:

    * cyfx_gcc_startup.S : Start-up code for the ARM-9 core on the FX3 device.
      This assembly source file follows the syntax for the GNU assembler.

    * cyfxtx.c           : ThreadX RTOS wrappers and utility functions required
      by the FX3 API library.

    * cyfxflashprog.h    : Constant definitions for the application.

    * cyfxusbdscr.c      : C source file containing the USB descriptors that
      are used by this firmware example. VID and PID is defined in this file.

    * cyfxflashprog.c    : C source file containing the code to initialize the
      I2C, SPI and USB blocks and implementing the vendor commands to read/write
      the EEPROM/FLASH devices.

    * makefile           : GNU make compliant build script for compiling this
      example.

   Vendor Commands implemented:

   1. Read Firmware ID
      bmRequestType = 0xC0
      bRequest      = 0xB0
      wValue        = 0x0000
      wIndex        = 0x0000
      wLength       = 0x0008

      Data response = "fx3prog"

   2. Write to I2C EEPROM
      bmRequestType = 0x40
      bRequest      = 0xBA
      wValue        = I2C EEPROM Slave Address (Can be in the 0 to 7 range, must be set according to the 
                      EEPROM address switch SW40)
      wIndex        = EEPROM byte address (can vary from 0x0000 to 0xFFFF. The max address is capped by 
                      the EEPROM max size)
      wLength       = Length of data to be written (Should be a multiple of 64 and less than or equal to 4096)

      Data phase should contain the actual data to be written.

   3. Read from I2C EEPROM
      bmRequestType = 0xC0
      bRequest      = 0xBB
      wValue        = I2C EEPROM Slave Address (Can be in the 0 to 7 range, must be set according to the 
                      EEPROM address switch SW40)
      wIndex        = EEPROM byte address (can vary from 0x0000 to 0xFFFF. The max address is capped by 
                      the EEPROM max size)
      wLength       = Length of data to be read (Should be a multiple of 64 and less than or equal to 4096)

      Data phase will contain the data read from the EEPROM

   4. Write to SPI flash
      bmRequestType = 0x40
      bRequest      = 0xC2
      wValue        = 0x0000
      wIndex        = SPI flash page address (Each page is assumed to be of 256 bytes and the byte address is
                      computed by multiplying wIndex by 256)
      wLength       = Length of data to be written (Should be a multiple of 256 and less than or equal to 4096)

      Data phase should contain the actual data to be written

   5. Read from SPI flash
      bmRequestType = 0xC0
      bRequest      = 0xC3
      wValue        = 0x0000
      wIndex        = SPI flash page address (Each page is assumed to be of 256 bytes and the byte address is
                      computed by multiplying wIndex by 256)
      wLength       = Length of data to be read (Should be a multiple of 256 and less than or equal to 4096)

      Data phase will contain the data read from the flash device

   6. Erase SPI flash sector
      bmRequestType = 0x40
      bRequest      = 0xC4
      wValue        = 0x0001
      wIndex        = SPI flash sector address (Each sector is assumed to be of 64 KB and the byte address is
                      computed by multiplying wIndex by 65536)
      wLength       = 0x0000

      No data phase is associated with this command.

   7. Check SPI busy status
      bmRequestType = 0xC0
      bRequest      = 0xC4
      wValue        = 0x0000
      wIndex        = 0x0000
      wLength       = 0x0001

      Data phase will indicate SPI flash busy status
      0x00 means SPI flash has finished write/erase operation and is ready for next command.
      Non-zero value means that SPI flash is still busy processing previous write/erase command.

Note:- bmRequestType, bRequest, wValue, wIndex, wLength are fields of setup packet. Refer USB 
       specification for understanding format of setup data. 
[]

