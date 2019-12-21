
#
# Use our local copy of the Cypress FX3 SDK.
#
FX3FWROOT=../../fx3-sdk
FX3PFWROOT=$(FX3FWROOT)/u3p_firmware
CYCONFOPT=fx3_release
include $(FX3FWROOT)/common/fx3_build_config.mak


# Specify the core sources for our firmware.
MODULE = exploration
SOURCE += \
	$(MODULE).c \
	usb_descriptors.c \
	application.c \
	spi_flash.c \
	sanity.c

# Set up the variables used by the Cypress makefiles.
C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)
EXES    = $(MODULE).elf

# By default, build a Cypress image.
all: $(MODULE).img

# Synthetic target: program the relevant image via the FX3 bootloader and pyfwup.
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
