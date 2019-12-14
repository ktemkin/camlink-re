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
### ARM Firmware configuration
###

# Tools
CY.ASM      = armasm
CY.CC       = armcc
CY.AR       = armar
CY.LD	    = armlink

# Arguments
ASMFLAGS	= -g --cpu ARM926EJ-S --apcs \interwork		\
			--pd "CYU3P_FX3 SETA (1)"

# If the build target is FPGA, make sure that CYU3P_FPGA is defined as part of ASMFLAGS
ifeq ($(CYCONFOPT), fx3_fpga_debug)
	ASMFLAGS += --pd "CYU3P_FPGA SETA (1)"
endif

CCFLAGS		+= --cpu ARM926EJ-S --apcs interwork

EXEEXT		= axf

LDFLAGS		+= -d --elf --remove				\
			--scatter $(FX3FWROOT)/common/cyfx3.scat	\
			--map --symbols --list $(MODULE).map \
			--no_strict_wchar_size --diag_suppress L6436W

# Command shortcuts
COMPILE		= $(CY.CC) $(CCFLAGS) -c -o $@ $<
ASSEMBLE	= $(CY.ASM) $(ASMFLAGS) -o $@ $<
LINK		= $(CY.LD) $(LDFLAGS) -o $@ $+
BDLIB		= $(CY.AR) --create $@ $+

#[]#
