##
## Copyright Cypress Semiconductor Corporation, 2010-2011,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed
## by the license agreement included in the file
##
##	<install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
##

###
### GNU toolchain Firmware configuration
###

# Tools
CC	= arm-none-eabi-gcc
AS	= arm-none-eabi-gcc
LD	= arm-none-eabi-ld
AR	= arm-none-eabi-ar

# Arguments
ASMFLAGS = -Wall -c -mcpu=arm926ej-s -mthumb-interwork

CCFLAGS += -Wall -mcpu=arm926ej-s -mthumb-interwork

# Select the linker script based on the chosen FX3 device.
ifeq ($(CYDEVICE), CYUSB3011)
    LDFLAGS += -T $(FX3FWROOT)/common/fx3_256k.ld 
else
    LDFLAGS += -T $(FX3FWROOT)/common/fx3_512k.ld
endif

LDFLAGS += -Wl,--gc-sections -Wl,--no-wchar-size-warning -Wl,-Map $(MODULE).map

EXEEXT		= elf

# Command Shortcuts
COMPILE		= $(CC) $(CCFLAGS) -c -o $@ $< 
ASSEMBLE	= $(AS) $(ASMFLAGS) -o $@ $<
LINK		= $(CC) $+ $(LDFLAGS) -o $@
BDLIB		= $(AR) -r $@ $+

# []
