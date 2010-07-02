#******************************************************************************
# @file      Makefile
# @author    Stefano Oliveri (software@stf12.net)
# @version   V2.0
# @date      22/06/2009
# @copy
#
# THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING USERS
# WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
# TIME. AS A RESULT, STEFANO OLIVERI SHALL NOT BE HELD LIABLE FOR ANY
# DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
# FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
# CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
#
# <h2><center>&copy; COPYRIGHT 2009 Stefano Oliveri</center></h2>
#******************************************************************************

# Project name
PROJECT_NAME=RTOSDemo

# Directory definition.

RTOS_SOURCE_DIR=../FreeRTOS/Source
ST_LIB_DIR=../Common/drivers/ST/STM32F10xFWLib/v3_1_2/Libraries/STM32F10x_StdPeriph_Driver
ST_EVAL_DIR=../Common/drivers/ST/STM32F10xFWLib/v3_1_2/Utilities/STM32_EVAL
ARM_CMSIS_DIR=../Common/drivers/ST/STM32F10xFWLib/v3_1_2/Libraries/CMSIS
EPL_DIR=../Common/ethernet/openPOWERLINK/EplStack
EPL_INCLUDE=../Common/ethernet/openPOWERLINK/Include
EPL_OBD=../Common/ethernet/openPOWERLINK/ObjDicts
EPL_EDRV=../Common/ethernet/openPOWERLINK/Edrv
EPL_TARGET=../Common/ethernet/openPOWERLINK/Target

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.).
OUTDIR = Debug

# Toolchain definition.
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-size
NM = arm-none-eabi-nm
LDSCRIPT=stm32_flash.ld


# should use --gc-sections but the debugger does not seem to be able to cope with the option.
LINKER_FLAGS=-nostartfiles -Xlinker -o$(PROJECT_NAME).axf -Xlinker -M -Xlinker -Map=$(PROJECT_NAME).map -Xlinker --no-gc-sections


# Debugging format.
#DEBUG = stabs
#DEBUG = dwarf-2
DEBUG= gdb

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 0



# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = gnu89


# Compiler flags definition.
CFLAGS=-g$(DEBUG) \
		-O$(OPT) \
		-std=$(CSTANDARD) \
		-T$(LDSCRIPT) \
		-I . \
		-I $(RTOS_SOURCE_DIR)/include \
		-I $(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3 \
		-I $(ST_LIB_DIR)/inc \
		-I $(ARM_CMSIS_DIR)/Core/CM3 \
		-I $(ST_EVAL_DIR) \
		-I $(ST_EVAL_DIR)/STM3210C_EVAL \
		-I $(EPL_INCLUDE) \
		-I $(EPL_INCLUDE)/kernel \
		-I $(EPL_INCLUDE)/user \
		-I $(EPL_OBD)/Api_CN \
		-I $(EPL_OBD)/Generic \
		-I $(EPL_TARGET)/STM32F107 \
		-D STM32F10X_CL \
		-D USE_STDPERIPH_DRIVER \
		-D VECT_TAB_FLASH \
		-D GCC_ARMCM3 \
		-D inline= \
		-mthumb \
		-mcpu=cortex-m3 \
		-ffunction-sections \
		-fdata-sections 

#CFLAGS += -D Debug

# Configuration files. When one of these file is modified, all source files are rebuilt.
CONFIG_HEADER= FreeRTOSConfig.h stm32f10x_conf.h Makefile

# Source files
SOURCE=	main.c \
		stm32f10x_it.c \
		stf_syscalls_minimal.c \
		ledTask.c \
		STCode/stm3210c_eval_lcd.c \
		drvTask.c \
		debugging.c \
		LedHandler.c \
		PushButton.c
		
		
# ST Library source files.		
ST_LIB_SOURCE= \
		$(ARM_CMSIS_DIR)/Core/CM3/core_cm3.c \
		$(ARM_CMSIS_DIR)/Core/CM3/system_stm32f10x.c \
		$(ST_LIB_DIR)/src/misc.c \
		$(ST_LIB_DIR)/src/stm32f10x_rcc.c \
		$(ST_LIB_DIR)/src/stm32f10x_flash.c \
		$(ST_LIB_DIR)/src/stm32f10x_gpio.c \
		$(ST_LIB_DIR)/src/stm32f10x_tim.c \
		$(ST_LIB_DIR)/src/stm32f10x_spi.c \
		$(ST_LIB_DIR)/src/stm32_eth.c
		
# FreeRTOS source files.		
FREERTOS_SOURCE= $(RTOS_SOURCE_DIR)/list.c \
		$(RTOS_SOURCE_DIR)/queue.c \
		$(RTOS_SOURCE_DIR)/tasks.c \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM3/port.c \
		$(RTOS_SOURCE_DIR)/portable/MemMang/heap_2.c
		
EPL_SOURCE = $(EPL_DIR)/amiarm.c \
		 $(EPL_DIR)/EplApiGeneric.c \
		 $(EPL_DIR)/EplApiProcessImage.c \
		 $(EPL_DIR)/EplDllk.c \
		 $(EPL_DIR)/EplDllkCal.c \
		 $(EPL_DIR)/EplDlluCal.c \
		 $(EPL_DIR)/EplErrorHandlerk.c \
		 $(EPL_DIR)/EplEventk.c \
		 $(EPL_DIR)/EplEventu.c \
		 $(EPL_DIR)/EplIdentu.c \
		 $(EPL_DIR)/EplLedu.c \
		 $(EPL_DIR)/EplNmtCnu.c \
		 $(EPL_DIR)/EplNmtk.c \
		 $(EPL_DIR)/EplNmtkCal.c \
		 $(EPL_DIR)/EplNmtMnu.c \
		 $(EPL_DIR)/EplNmtu.c \
		 $(EPL_DIR)/EplNmtuCal.c \
		 $(EPL_DIR)/EplObd.c \
		 $(EPL_DIR)/EplObdkCal.c \
		 $(EPL_DIR)/EplObdu.c \
		 $(EPL_DIR)/EplObduCal.c \
		 $(EPL_DIR)/EplPdok.c \
		 $(EPL_DIR)/EplPdokCal.c \
		 $(EPL_DIR)/EplPdouCal.c \
		 $(EPL_DIR)/EplPdou.c \
		 $(EPL_DIR)/EplSdoAsndu.c \
		 $(EPL_DIR)/EplSdoAsySequ.c \
		 $(EPL_DIR)/EplSdoComu.c \
		 $(EPL_DIR)/EplSdoUdpu.c \
		 $(EPL_DIR)/EplStatusu.c \
		 $(EPL_DIR)/EplTimeruGeneric.c \
		 $(EPL_DIR)/EplApiFreeRTOS.c \
		 $(EPL_OBD)/Api_CN/Objdict.c \
		 $(EPL_EDRV)/EdrvF107.c \
		 $(EPL_TARGET)/STM32F107/STM32Timer.c	 

SOURCE+=$(ST_LIB_SOURCE)		
SOURCE+=$(FREERTOS_SOURCE)
SOURCE +=$(EPL_SOURCE)

# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(SOURCE)))


LIBS= 

# List of all objects files.
OBJS = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))


# Define Messages.
# English
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------


# Rules definition. ***********************************************************

all: begin gccversion $(OUTDIR)/$(PROJECT_NAME).bin end
	 
$(OUTDIR)/$(PROJECT_NAME).bin : $(PROJECT_NAME).axf
	$(OBJCOPY) $(PROJECT_NAME).axf -O binary $(PROJECT_NAME).bin

$(PROJECT_NAME).axf : $(OBJS) $(OUTDIR)/startup_stm32f10x.o Makefile
	$(CC) $(CFLAGS) $(OBJS) $(OUTDIR)/startup_stm32f10x.o $(LIBS) $(LINKER_FLAGS)

$(OBJS) :  $(CONFIG_HEADER)

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
##	@echo
	@echo $$< "->" $$@
	$(CC) -c  $$(CFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SOURCE), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 	

$(OUTDIR)/startup_stm32f10x.o : startup_stm32f10x.c Makefile
	$(CC) -c $(CFLAGS) -O1 startup_stm32f10x.c -o $(OUTDIR)/startup_stm32f10x.o
		
clean :
	-cs-rm $(OBJS)
	-cs-rm $(OUTDIR)/startup_stm32f10x.o
	-cs-rm $(PROJECT_NAME).axf
	-cs-rm $(PROJECT_NAME)_SymbolTable.txt  
	-cs-rm $(PROJECT_NAME)_MemoryListingSummary.txt
	cs-rm $(PROJECT_NAME)_MemoryListingDetails.txt

log : $(PROJECT_NAME).axf
	$(NM) -n $(PROJECT_NAME).axf > $(PROJECT_NAME)_SymbolTable.txt
	$(OBJDUMP) --format=SysV $(PROJECT_NAME).axf > $(PROJECT_NAME)_MemoryListingSummary.txt
	$(OBJDUMP) $(OBJS) > $(PROJECT_NAME)_MemoryListingDetails.txt
	
# Eye candy.
begin:
##	@echo
	@echo $(MSG_BEGIN)
		
end:
	@echo $(MSG_END)
##	@echo
		
# Display compiler version information.
gccversion : 
	@$(CC) --version
		
$(shell mkdir $(OUTDIR) 2>NUL)
