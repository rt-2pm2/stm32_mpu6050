# Makefile for the project 
#
#
.PHONY: list clear assemble flash

CC = arm-none-eabi-gcc
CC_OPTIONS = -g -O0
device = stm32f407xx

frw_name = main

home = $(shell pwd)


##### ============================== ######
##-      Collecting the file lists

# Folder containing the system source files
include_root := ./include
src := ./src
src_core := $(src)/system
src_hal := $(include_root)/STM32F4xx_HAL_Driver/Src
src_dev := $(include_root)/MPU6050

# Core Source files
core_src_files := $(wildcard $(src_core)/*.c)
core_src_files_o := $(patsubst %.c, %.o, $(core_src_files))

# HAL Source files
hal_src_files := stm32f4xx_hal.c stm32f4xx_hal_flash.c \
 stm32f4xx_hal_flash_ex.c stm32f4xx_hal_flash_ramfunc.c \
 stm32f4xx_hal_cortex.c stm32f4xx_hal_i2c.c stm32f4xx_hal_dma.c \
 stm32f4xx_hal_gpio.c stm32f4xx_hal_rcc.c stm32f4xx_hal_uart.c
hal_src_files_o := $(patsubst %.c, %.o, $(hal_src_files))

dev_src_files := $(wildcard $(src_dev)/*.c)
dev_src_files_o := $(patsubst %.c, %.o, $(dev_src_files))

# Startup assembly file
startup_s := $(src_core)/startup_$(device).s
startup_o := $(patsubst %.s, %.o, $(startup_s))

# Application Source files
app_src_files := $(wildcard $(src)/*.c)
app_src_files_o := $(patsubst %.c, %.o, $(app_src_files))

# 
# Compose the total list
objfiles := $(core_src_files_o) $(startup_o) $(app_src_files_o) $(dev_src_files_o)


##-      Header files and folders 

# Headers files of the core
core_hs = $(shell find $(include_root) -type f -name "*.h")

# List of directories containing the header files
core_inc_d = $(shell dirname $(core_hs) | sort -u)

# List of directories containing the header files with the "-I" prefix
core_inc_d_I = $(foreach dir, $(core_inc_d), -I$(dir))


##-      Linker script
link_script = $(wildcard $(src_core)/*.ld)


all: main.hex

assemble: main.hex 

main.hex: main.elf
	arm-none-eabi-objcopy -Oihex $< $@
	@echo "Done"

main.elf: $(objfiles) $(hal_src_files_o)
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(eval hal_built := $(foreach file, $(hal_src_files_o), ./build/hal/$(file)))
	$(CC) -mcpu=cortex-m4 -mlittle-endian -mthumb -DSTM32F407xx \
		-T$(link_script) -Wl,--gc-sections $(CC_OPTIONS)\
		$(objfiles) $(hal_built)\
		-o $@
	@echo " ========= Done Compiling " $@ " ========= "

$(app_src_files_o): %.o: %.c 
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(CC) -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb $(CC_OPTIONS)\
		$(core_inc_d_I) -DSTM32F407xx -c $< -o $@
	@echo " ========= Done Compiling " $@ " ========= "


$(core_src_files_o): %.o: %.c 
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(CC) -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb $(CC_OPTIONS)\
		$(core_inc_d_I) -DSTM32F407xx -c $< -o $@
	@echo " ========= Done Compiling " $@ " ========= "

$(hal_src_files_o): %.o : ./include/STM32F4xx_HAL_Driver/Src/%.c
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(CC) -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb $(CC_OPTIONS)\
		$(core_inc_d_I) -DSTM32F407xx -c $< \
		-o ./build/hal/$@
	@echo " ========= Done Compiling " $@ " ========= "

$(dev_src_files_o): %.o: %.c 
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(CC) -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb $(CC_OPTIONS)\
		$(core_inc_d_I) -DSTM32F407xx -c $< -o $@
	@echo " ========= Done Compiling " $@ " ========= "


$(startup_o): %.o: %.s 
	@echo -e " \n ========= Compiling " $@ " ========= "
	$(CC) -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb $(CC_OPTIONS)\
		$(core_inc_d_I) -DSTM32F407xx -c $< -o $@
	@echo " ========= Done Compiling " $@ " ========= "

flash: main.elf
	$(shell openocd -f ~/usr/openocd/tcl/board/stm32f429discovery.cfg -c "program main.elf verify reset exit")


list: 
#	@echo "Base folder: " $(home)
#	@echo "Core Header Include directories: " $(core_inc_d)
#	@echo "Core Include par: " $(core_inc_d_I)
#	@echo "core_src_files: " $(core_src_files)
#	@echo "core_src_files_o: " $(core_src_files_o)

	@echo "hal_src_files_o: " $(hal_src_files_o)

clear:
	rm $(objfiles) ./build/hal/*
