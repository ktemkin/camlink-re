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
##      <install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
##

FX3FWROOT=../../fx3-sdk
FX3PFWROOT=$(FX3FWROOT)/u3p_firmware


include $(FX3FWROOT)/common/fx3_build_config.mak

MODULE = exploration

SOURCE += $(MODULE).c 
SOURCE += cyfxusbdscr.c 

C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)

EXES = $(MODULE).elf

all: $(MODULE).img

program: $(MODULE).img
	fx3load $<

$(MODULE).img: $(MODULE).elf
	elf2img -i $< -o $@ -vectorload yes

$(MODULE).elf: $(A_OBJECT) $(C_OBJECT)
	$(LINK)

$(C_OBJECT) : %.o : %.c
	$(COMPILE)

$(A_OBJECT) : %.o : %.S
	$(ASSEMBLE)

clean:
	rm -f ./$(MODULE).img
	rm -f ./$(MODULE).elf
	rm -f ./$(MODULE).map
	rm -f ./*.o


compile: $(C_OBJECT) $(A_OBJECT) $(EXES)

#[]#
