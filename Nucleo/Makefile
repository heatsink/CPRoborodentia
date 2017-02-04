########################################################
# Makefile by CubeMX2Makefile.py
########################################################
# Path to gcc
GCC_PATH = /usr/bin/
# Path to st-flash tool
FLASH_PATH = /usr/local/bin/st-flash

########################################################
# target
########################################################
TARGET = Nucleo

########################################################
# building variables
########################################################
# optimization
OPT = -O2

#########################################################
# pathes
#########################################################
# source path
# Build path
BUILD_DIR = build

########################################################
# source
########################################################
C_SOURCES = \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
  Src/main.c \
  Src/stm32f4xx_hal_msp.c \
  Src/stm32f4xx_it.c \
  Src/system_stm32f4xx.c  
ASM_SOURCES = \
  SW4STM32/startup_stm32f446xx.s

#######################################
# binaries
#######################################
#PREFIX  = $(GCC_PATH)arm-none-eabi-
PREFIX  = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#########################################################
# CFLAGS
#########################################################
# macros for gcc
AS_DEFS =
C_DEFS =  -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IInc
C_INCLUDES += -IDrivers/STM32F4xx_HAL_Driver/Inc
C_INCLUDES += -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy
C_INCLUDES += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
C_INCLUDES += -IDrivers/CMSIS/Include
# compile gcc flags
WFLAGS = -Wall -fdata-sections -ffunction-sections -fmessage-length=0 -c
ASFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp $(AS_DEFS) $(AS_INCLUDES) $(OPT) $(WFLAGS)
CFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fno-aggressive-loop-optimizations $(C_DEFS) $(C_INCLUDES) $(OPT) $(WFLAGS)
# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

#########################################################
# LDFLAGS
#########################################################
# link script
LDSCRIPT = "SW4STM32/Nucleo/STM32F446RETx_FLASH.ld"
# libraries
LIBS = -lc -lm -lnosys
LIBS += 
LIBDIR =
CC_VER := $(shell $(CC) -dumpversion)
ifeq "$(CC_VER)" "4.8.2"
	LDSPECS = -specs=nosys.specs 
else
	LDSPECS = -specs=nano.specs -specs=nosys.specs 
endif
LDFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp $(LDSPECS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#########################################################
# build the application
#########################################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "C. Compiling $@..."
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "S. Compiling $@..."
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "C. Linking $@..."
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "H. Linking $@..."
	@$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "B. Building $@..."
	@$(BIN) $< $@	
	@echo "Used gcc: "$(CC_VER)
	
$(BUILD_DIR):
	mkdir -p $@		

#########################################################
# clean up
#########################################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#########################################################
# Code::Blocks commands:
#########################################################
Release: all flash
Debug: CFLAGS += -g -gdwarf-2
Debug: all
cleanDebug: clean
cleanRelease: clean
flash:
	$(FLASH_PATH) erase
	$(FLASH_PATH) --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

#########################################################
# dependencies
#########################################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
