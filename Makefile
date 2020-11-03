CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
LIBOPENCM3 ?= ./libopencm3
GIT_VERSION := $(shell git describe --abbrev=8 --dirty --always --tags)

# Config bits
APP_ADDRESS = 0x08001000

PLATFORM_DEFS = -DSTM32F1 -mthumb -mcpu=cortex-m3 -DTARGET_STM32PROG

CFLAGS = -O2 -std=c11 -Wall -pedantic -Werror -Istm32/include \
	-ffunction-sections -fdata-sections -Wno-address-of-packed-member \
	-I$(LIBOPENCM3)/include -DAPP_ADDRESS=$(APP_ADDRESS)   \
	-ggdb -DVERSION=\"$(GIT_VERSION)\" -flto \
	$(PLATFORM_DEFS)

LDFLAGS = -lopencm3_stm32f1 -ggdb \
	-ffunction-sections -fdata-sections \
	-Wl,-Tstm32f103.ld -nostartfiles -lnosys \
	-L$(LIBOPENCM3)/lib/ -Wl,-gc-sections -flto \
	-Wl,--print-memory-usage \
	$(PLATFORM_DEFS)

all:	magicmouse.bin client.exe

magicmouse.elf: main.o | $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) $^ -o $@ $(LDFLAGS) -Wl,-Ttext=$(APP_ADDRESS) -Wl,-Map,magicmouse.map

$(LIBOPENCM3)/lib/libopencm3_stm32f1.a:
	$(MAKE) -C $(LIBOPENCM3) TARGETS=stm32/f1 AR=$(CC)-ar CFLAGS=-flto LDFLAGS=-flto

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@
	python3 stm32-bootloader/checksum.py $@

%.o: %.c $(LIBOPENCM3)/lib/libopencm3_stm32f1.a
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	-rm -f *.elf *.o *.bin *.map *.exe

client.exe:	client.cc
	g++ -o client.exe client.cc -lusb-1.0 -ggdb

