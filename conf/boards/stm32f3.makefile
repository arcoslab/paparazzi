# Hey Emacs, this is a -*- makefile -*-
#
# stm32f3.makefile
#
# 
#

BOARD=stm32f3
BOARD_VERSION=discovery
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f3
HARD_FLOAT=1
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny
$(TARGET).LDSCRIPT=$(SRC_ARCH)/stm32f3.ld

# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = DFU
#FLASH_MODE = JTAG
#FLASH_MODE = SERIAL
endif

ifndef NO_LUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8000000
endif

#
#
# some default values shared between different firmwares
#
#


#
# default LED configuration
#
ifndef RADIO_CONTROL_LED
RADIO_CONTROL_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = 2
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = 1
endif

#
# default uart configuration
#
ifndef RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = USART1
endif

ifndef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT = USART2
endif

ifndef MODEM_PORT
MODEM_PORT=UART5
endif

ifndef MODEM_BAUD
MODEM_BAUD=B57600
endif

