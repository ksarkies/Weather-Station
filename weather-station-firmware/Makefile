# Basic makefile K Sarkies

PROJECT		= weather-station
PREFIX		= arm-none-eabi
CC			= $(PREFIX)-gcc
LD			= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
GDB			= $(PREFIX)-gdb
NM 			= $(PREFIX)-nm

MCU         = cortex-m3
PROC        = STM32F1
CHIP        = STM32F10x_128k_20k
BOARD       = ET_STAMP_STM32
F_XTAL      = 8000000
SYSCLOCK_CL = SYSCLK_FREQ_72MHz=72000000

LIBRARY_DIR     = /home/ksarkies/Development-Software/arm-library
DRIVERS_DIR	    = $(LIBRARY_DIR)/libopencm3-examples/libopencm3
DRIVERS_SRC	    = $(DRIVERS_DIR)/lib/stm32/f1
DRIVERS_INC	    = $(DRIVERS_DIR)/include
LIBS_DIR        = ../libs
FATFSDIR        = ../chan-fat-stm32-loc3

VPATH       += $(DRIVERS_SRC)/ $(DRIVERS_SRC)/../ $(FATFSDIR) $(LIBS_DIR)

# Inclusion of header files
INCLUDES	= $(patsubst %,-I%,$(DRIVERS_INC) $(FATFSDIR))

CDEFS       += -DHSE_VALUE=$(F_XTAL)UL
CDEFS       += -D$(SYSCLOCK_CL)
CDEFS       += -DUSE_$(BOARD)
CDEFS       += -DVERSION=$(VERSION)

CFLAGS	    += -Os -g -Wall -Wextra -Wno-unused-variable -I. $(INCLUDES) \
               -fno-common -mthumb -MD
CFLAGS	    += -DGCC_ARMCM3 -ffunction-sections -fdata-sections
CFLAGS	    += -mcpu=$(MCU) -D$(PROC) -msoft-float
CFLAGS	    += $(CDEFS)

LDSCRIPT    = stm32-h103RET6.ld

LDFLAGS	    += -I . -lc -T$(LDSCRIPT) -L$(DRIVERS_DIR)/lib -nostartfiles \
               -Wl,--gc-sections
LDFLAGS	    += -lopencm3_stm32f1
LDFLAGS	    += -specs=nosys.specs

# The libopencm3 library is assumed to exist in libopencm3/lib, otherwise add files here
CFILES		= $(PROJECT).c $(PROJECT)-objdic.c
CFILES      += hardware.c dht.c i2clib.c barosensor.c
CFILES      += buffer.c comms.c file.c timelib.c
CFILES      += ff.c sd_spi_loc3_stm32.c fattime.c

OBJS		= $(CFILES:.c=.o)

all: $(PROJECT).elf $(PROJECT).bin $(PROJECT).hex $(PROJECT).list $(PROJECT).sym

$(PROJECT).elf: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex  $< $@

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary  $< $@

$(PROJECT).list: $(PROJECT).elf
	$(OBJDUMP) -S $< > $@

$(PROJECT).sym: $(PROJECT).elf
	$(NM) -n $< > $@

clean:
	rm *.elf *.o *.d *.hex *.list *.sym *.bin ../libs/*.o ../libs/*.d


