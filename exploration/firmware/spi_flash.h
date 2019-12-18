/*
## Cypress code for working with the primary SPI flash.
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 2010-2011,
##  All Rights Reserved. // UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed by the license agreement
##  included in the file CYPRESS_LICENSE.txt
##
## ===========================
*/

#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include <cyu3types.h>

int spi_flash_initialize(uint16_t pageLen);
int spi_flash_program(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead);
int spi_flash_erase_sector(bool isErase, uint8_t sector, uint8_t *wip);
void spi_flash_reset(void);

#endif
