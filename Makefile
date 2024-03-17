# ------------------------------------------------
# Generic Makefile (based on gcc)
# ------------------------------------------------

######################################
# target
######################################
TARGET = main

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
OUTPUT_DIR = output

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

######################################
# source
######################################
# ASM sources
ASM_SOURCES =  \
	Device/STM32F10x/startup/startup_stm32f10x_md.s
$(info ASM sources files: [ $(ASM_SOURCES) ])

# C sources
C_SOURCES =  \
	$(wildcard Device/cortex-m3/*.c) \
	$(wildcard Device/STM32F10x/*.c) \
	$(wildcard Device/STM32F10x_StdPeriph_Driver/*.c) \
	$(wildcard System/*.c) \
	$(wildcard User/*.c)
$(info C sources files: [ $(C_SOURCES) ])

# CPP sources
CPP_SOURCES =  \
	$(wildcard User/*.cpp)

$(info CPP sources files: [ $(CPP_SOURCES) ])


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
XX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
DP = $(GCC_PATH)/$(PREFIX)objdump
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
XX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
DP = $(PREFIX)objdump
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# C_FLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# 闪存起始地址，对于STM32设备通常是固定的。
FLASHSTART = 0x08000000

# fpu
# NONE for Cortex-M0/M0+/M3
FPU = 

# float-abi
FLOAT-ABI = 

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_CPP_DEFS =  \
	-D STM32F10X_MD\
	-D USE_STDPERIPH_DRIVER

# AS includes
AS_INCLUDES = 

# C includes
C_CPP_INCLUDES =  \
	-IDevice/cortex-m3 \
	-IDevice/STM32F10x \
	-IDevice/STM32F10x_StdPeriph_Driver \
	-ISystem \
	-IUser


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

C_FLAGS += $(MCU) $(C_CPP_DEFS) $(C_CPP_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# -fno-rtt 表示不使用RTTI，即运行时类型识别，这意味着代码中不能使用typeid和dynamic_cast，但这可以使编译后的固件文件减小。
# -fno-exceptions 表示不捕获程序异常，此选项也可以减小编译后的固件文件大小。 
CPP_FLAGS += $(MCU) $(C_CPP_DEFS) $(C_CPP_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions

ifeq ($(DEBUG), 1)
C_FLAGS += -g -gdwarf-2
CPP_FLAGS += -g -gdwarf-2
endif


# Generate dependency information
C_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CPP_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# build the application
#######################################
# list of C objects
C_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of CPP objects
CPP_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of ASM program objects
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR):
	mkdir $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo CC $<
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	@echo XX $<
	$(XX) -c $(CPP_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo AS $<
	$(AS) -c $(C_FLAGS) $< -o $@

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Linker/STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections,--print-memory-usage

#######################################
# Linker
#######################################
$(OUTPUT_DIR):
	mkdir $@

$(OUTPUT_DIR)/$(TARGET).elf: $(ASM_OBJECTS) $(C_OBJECTS) $(CPP_OBJECTS) Makefile | $(OUTPUT_DIR)
	@echo LD $@
	$(CC) $(ASM_OBJECTS) $(C_OBJECTS) $(CPP_OBJECTS) $(LDFLAGS) -o $@
	@echo SIZE $@
	$(SZ) $@

$(OUTPUT_DIR)/$(TARGET).hex: $(OUTPUT_DIR)/$(TARGET).elf | $(OUTPUT_DIR)
	@echo Creating hex file: $@
	$(HEX) $< $@
	
$(OUTPUT_DIR)/$(TARGET).bin: $(OUTPUT_DIR)/$(TARGET).elf | $(OUTPUT_DIR)
	@echo Creating bin file: $@
	$(BIN) $< $@
	$(ASMOUTPUTFILE)

$(OUTPUT_DIR)/$(TARGET).s: $(OUTPUT_DIR)/$(TARGET).elf | $(OUTPUT_DIR)
	$(DP) -DSG -t -marm -w --show-raw-insn --start-address=$(FLASHSTART) \
	--visualize-jumps --inlines $< \
	-Mforce-thumb -Mreg-names-std > $@


#######################################
# build all
#######################################
# default action: build all
all: $(OUTPUT_DIR)/$(TARGET).elf $(OUTPUT_DIR)/$(TARGET).hex $(OUTPUT_DIR)/$(TARGET).bin $(OUTPUT_DIR)/$(TARGET).s

#######################################
# write
#######################################
write: $(OUTPUT_DIR)/$(TARGET).bin
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f1x.cfg \
		-c "init; reset halt; wait_halt; flash write_image erase $(OUTPUT_DIR)/$(TARGET).bin ${FLASHSTART}; reset; shutdown;" 
		@echo "Write Completed."

#######################################
# erase
#######################################
erase:
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f1x.cfg \
		-c "init; reset halt; flash erase_sector 0 0 1; exit"
	@echo "Erase Completed."

#######################################
# reset
#######################################
reset:
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f1x.cfg \
		-c "init; reset; exit"
	@echo "Erase Completed."
	
#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
	-rm -fR $(OUTPUT_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***