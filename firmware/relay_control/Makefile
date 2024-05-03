MCU = attiny45
FREQ = 8000000
OPT = 2
USB_PORT ?= 0
CONFIG ?= 

SRC_DIR = .
TARGET = relay_control
TARGET_DIR = ../out/$(TARGET)
FORMAT = ihex

CDEFS += -DF_CPU=$(FREQ) $(patsubst %,-DCONFIG_%,$(CONFIG))
CFLAGS = -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-stack-protector -fdata-sections -ffunction-sections --param=min-pagesize=0 -Wall -Wextra -Wstrict-prototypes -g -O$(OPT) -Wa,-adhlns=$(addprefix $(TARGET_DIR)/,$(notdir $(<:.c=.lst)))
LDFLAGS = -Wl,-Map=$(TARGET_DIR)/$(TARGET).map,--cref,--gc-sections -lm

AVRDUDE_PROGRAMMER = avrisp
AVRDUDE_BAUD = 19200
AVRDUDE_PORT = /dev/ttyUSB$(USB_PORT)    # programmer connected to serial device

# ---------------------------------------------------------------------------

# Define programs and commands.
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -r
MKDIR = mkdir -p

# Collect source and intermediary files
SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(addprefix $(TARGET_DIR)/,$(notdir $(SRC:.c=.o)))

# Compiler flags to generate dependency files.
GENDEPFLAGS = -Wp,-M,-MP,-MT,$(TARGET_DIR)/$(*F).o,-MF,$(TARGET_DIR)/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I$(SRC_DIR) $(CFLAGS) $(CDEFS) $(GENDEPFLAGS)
ALL_LDFLAGS = $(LDFLAGS)

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET_DIR)/$(TARGET).hex
AVRDUDE_FLAGS = -p $(MCU) -c $(AVRDUDE_PROGRAMMER) -P $(AVRDUDE_PORT) -b $(AVRDUDE_BAUD)

# ---------------------------------------------------------------------------

# Default target.
all: build elfsize

build: $(TARGET_DIR)/$(TARGET).elf $(TARGET_DIR)/$(TARGET).hex $(TARGET_DIR)/$(TARGET).lss $(TARGET_DIR)/$(TARGET).sym

# Display size of file.
size: elfsize hexsize

elfsize: $(TARGET_DIR)/$(TARGET).elf 
	$(SIZE) -A $<

hexsize: $(TARGET_DIR)/$(TARGET).hex
	$(SIZE) --target=$(FORMAT) $^

# Program the device
program: $(TARGET_DIR)/$(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

# Target: clean project
clean: 
	$(REMOVE) $(TARGET_DIR)

# ---------------------------------------------------------------------------

# Create output directory
$(TARGET_DIR):
	$(MKDIR) $(TARGET_DIR)

# Create final output files (.hex) from ELF output file
$(TARGET_DIR)/%.hex: $(TARGET_DIR)/%.elf | $(TARGET_DIR)
	$(OBJCOPY) -O $(FORMAT) -j .text -j .data $< $@

# Create extended listing file from ELF output file
$(TARGET_DIR)/%.lss: $(TARGET_DIR)/%.elf | $(TARGET_DIR)
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file
$(TARGET_DIR)/%.sym: $(TARGET_DIR)/%.elf | $(TARGET_DIR)
	$(NM) -n $< > $@

# Link: create ELF output file from object files
$(TARGET_DIR)/%.elf: $(OBJ) | $(TARGET_DIR)
	$(CC) $(ALL_CFLAGS) -o $@ $(OBJ) $(ALL_LDFLAGS)

# Compile: create object files from C source files
$(TARGET_DIR)/%.o : $(SRC_DIR)/%.c | $(TARGET_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

# Include the dependency files
include $(wildcard $(TARGET_DIR)/*.d)

# Listing of phony targets
.PHONY: all size elfsize hexsize build elf hex eep lss sym clean program program_flash program_eeprom
.SECONDARY: $(TARGET_DIR)/$(TARGET).elf
.PRECIOUS: $(OBJ)

