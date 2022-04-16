##############################################################################
# Project, sources and paths
#

PROJECT = blinky

CSRC = main.c gpio.c timer.c selector.c
#CSRC = main.c gpio.c motor.c
#CSRC = main.c gpio.c timer.c motor.c
CSRC += ../ST/system_clock_config.c ../ST/stm32f4xx_ll_rcc.c ../ST/system_stm32f4xx.c

ASMSRC = ../ST/startup_stm32f407xx.s

INCDIR = . ../ST

LIBS =
LIBDIR =

LDSCRIPT = ../ST/STM32F407VGTx_FLASH.ld

DEFS = -DSTM32F4 -DSTM32F407xx

OBJS = $(CSRC:.c=.o) $(ASMSRC:.s=.o)
IINCDIR = $(patsubst %,-I%,$(INCDIR))
LLIBDIR = $(patsubst %,-L%,$(LIBDIR))

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
LD   = $(TRGT)gcc
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
NM   = $(TRGT)nm

# Compiler options
OPT = -O0 -ggdb -fomit-frame-pointer -falign-functions=16
OPT += -ffunction-sections -fdata-sections -fno-common

# THUMB-specific options
TOPT = -mthumb -mno-thumb-interwork

# Define C warning options
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

CFLAGS = -mcpu=$(MCU) $(OPT) $(CWARN) $(DEFS) $(TOPT) -MD -MP

ASFLAGS = -mcpu=$(MCU) $(TOPT)

LDFLAGS = -mcpu=$(MCU) $(OPT) -nostartfiles $(LLIBDIR) $(TOPT)
LDFLAGS += -Wl,--no-warn-mismatch,--gc-sections,--script=$(LDSCRIPT)

##############################################################################
# Make targets
#

.PHONY: all
all: $(PROJECT).elf $(PROJECT).list $(PROJECT).size $(PROJECT).mem
	$(SZ) $(PROJECT).elf
	@ echo "> Done"

.PHONY: clean
clean:
	@echo "> Cleaning"
	rm -f $(OBJS)
	rm $(CSRC:%.c=%.d)
	rm -f $(PROJECT).elf $(PROJECT).list $(PROJECT).size $(PROJECT).mem
	@echo "> Done"

%.o: %.c Makefile
	@echo "> Compiling" $(<F)
	$(CC) -c $(CFLAGS) $(IINCDIR) $< -o $@

%.o: %.s Makefile
	@echo "> Compiling" $(<F)
	$(AS) -c $(ASFLAGS) $(IINCDIR) $< -o $@

$(PROJECT).elf: $(OBJS) $(LDSCRIPT)
	@echo "> Linking" $@
	$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

$(PROJECT).list: $(PROJECT).elf
	@echo "> Creating" $@
	$(OD) -d $< > $@

$(PROJECT).size: $(PROJECT).elf
	@echo "> Creating" $@
	$(NM) --size-sort --print-size $< > $@

$(PROJECT).mem: $(PROJECT).elf
	@echo "> Creating" $@
	$(NM) --numeric-sort --print-size $< > $@

.PHONY: flash
flash: all
	arm-none-eabi-gdb --command=../../debug.gdb --command=../../flash.gdb blinky.elf

-include $(CSRC:%.c=%.d)