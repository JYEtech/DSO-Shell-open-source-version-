#
# WinARM template makefile 
#
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make program = Upload load-image to the device
#
# make filename.s = Just compile filename.c into the assembler code only
#
# make filename.o = Create object filename.o from filename.c (using CFLAGS)
#
# To rebuild project do "make clean" then "make all".
#

# This should work with just CS G++ lite installed (Win32 cmd as shell, cs-make, cs-rm)
# Tested with: 
# - CS G++ 2009-Q1 (cs-make 3.81, cs-rm, cmd.exe), no MinGW or Cygwin tools in the $PATH
# - Cygwin (make 3.81, rm, bash 3.2.39)
# - MSYS/MinGW tools in $(PATH) (WinAVR\utils\bin)


# Toolchain prefix (i.e arm-elf- -> arm-elf-gcc.exe)
#TCHAIN_PREFIX = arm-eabi-
#TCHAIN_PREFIX = arm-elf-
TCHAIN_PREFIX = arm-none-eabi-
#REMOVE_CMD=rm
REMOVE_CMD=cs-rm

#FLASH_TOOL = OPENOCD
#FLASH_TOOL = LPC21ISP
#FLASH_TOOL = UVISION

# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC and -mthumb-interwork option for all source
USE_THUMB_MODE = YES
#USE_THUMB_MODE = NO

# MCU name, submodel and board
# - MCU used for compiler-option (-mcpu)
# - SUBMDL used for linker-script name (-T) and passed as define
# - BOARD just passed as define (optional)
MCU      = cortex-m3
CHIP     = STM32F10x_64k_20k
BOARD    = JYE109-15000-00D
F_XTAL   = 8000000
SYSCLOCK_CL = SYSCLK_FREQ_72MHz=72000000

# *** This example only supports "FLASH_RUN" ***
# RUN_MODE is passed as define and used for the linker-script filename,
# the user has to implement the necessary operations for 
# the used mode(s) (i.e. no copy of .data, remapping)
# Create FLASH-Image
RUN_MODE=FLASH_RUN
# Create RAM-Image
#RUN_MODE=RAM_RUN

# Exception-vectors placement option is just passed as define,
# the user has to implement the necessary operations (i.e. remapping)
# Exception vectors in FLASH:
ifeq ($(RUN_MODE),RAM_RUN)
VECTOR_TABLE_LOCATION=VECT_TAB_RAM
else 
VECTOR_TABLE_LOCATION=VECT_TAB_FLASH
endif

#VECTOR_TABLE_LOCATION=VECT_TAB_FLASH
# Exception vectors in RAM:
#VECTOR_TABLE_LOCATION=VECT_TAB_RAM

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR = $(RUN_MODE)

# Target file name (without extension).
TARGET = 113-15011-060

# Pathes to libraries
APPLIBDIR = ./Libraries
STMLIBDIR = $(APPLIBDIR)
STMSPDDIR = $(STMLIBDIR)/STM32F10x_StdPeriph_Driver
STMSPDSRCDIR = $(STMSPDDIR)/src
STMSPDINCDIR = $(STMSPDDIR)/inc
CMSISDIR  = $(STMLIBDIR)/CMSIS/CM3/CoreSupport

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
## Demo-Application:
SRC = 113-15011.c Board.c Common.c Screen.c Command.c stm32f10x_it.c Eeprom.c

## compiler-specific sources
#SRC += startup_stm32f10x_md_mthomas.c
## CMSIS for STM32
SRC += $(CMSISDIR)/core_cm3.c
SRC += $(CMSISDIR)/system_stm32f10x.c
## used parts of the STM-Library
SRC += $(STMSPDSRCDIR)/misc.c
SRC += $(STMSPDSRCDIR)/stm32f10x_gpio.c
SRC += $(STMSPDSRCDIR)/stm32f10x_usart.c
SRC += $(STMSPDSRCDIR)/stm32f10x_rcc.c
SRC += $(STMSPDSRCDIR)/stm32f10x_flash.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_spi.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_rtc.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_bkp.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_pwr.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_dma.c
SRC += $(STMSPDSRCDIR)/stm32f10x_tim.c
#SRC += $(STMSPDSRCDIR)/stm32f10x_fsmc.c

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
SRCARM = 

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
#CPPSRC = main.cpp myclasses.cpp
#CPPSRC += mini_cpp.cpp

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = startup_stm32f10x_md.S

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM  =

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  = $(STMSPDINCDIR) $(CMSISDIR) $(FATSDDIR) $(MININIDIR) $(STMEEEMULINCDIR)
EXTRAINCDIRS += $(APPLIBDIR) $(SWIMSRCDIR)
EXTRAINCDIRS += C:/Engineering/SourceryCodeBenchLite/arm-none-eabi/include
EXTRAINCDIRS += ./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
# Extra libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS = dso150

# Path to Linker-Scripts
LINKERSCRIPTPATH = .
LINKERSCRIPTINC  = .

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = $(LINKERSCRIPTINC)

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
#OPT = s
OPT = 2
#OPT = 3
#OPT = 0

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format i.e. for lpc21isp
#LOADFORMAT = ihex
#LOADFORMAT = binary
LOADFORMAT = both

# Using the Atmel AT91_lib produces warnings with
# the default warning-levels. 
#  yes - disable these warnings
#  no  - keep default settings
#AT91LIBNOWARN = yes
AT91LIBNOWARN = no

# Debugging format.
#DEBUG = stabs
#DEBUG = dwarf-2
DEBUG = gdb

# Place project-specific -D (define) and/or 
# -U options for C here.
CDEFS = -DSTM32F10X_MD
CDEFS += -DHSE_VALUE=$(F_XTAL)UL
CDEFS += -D$(SYSCLOCK_CL)
CDEFS += -DUSE_STDPERIPH_DRIVER
CDEFS += -DUSE_$(BOARD)
CDEFS += -DSTM32_SD_USE_DMA
CDEFS += -DSTARTUP_DELAY
# enable modifications in STM's libraries
CDEFS += -DMOD_MTHOMAS_STMLIB
# enable parameter-checking in STM's library
CDEFS += -DUSE_FULL_ASSERT

# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
#ADEFS = -DUSE_IRQ_ASM_WRAPPER
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

#-----

ifdef VECTOR_TABLE_LOCATION
CDEFS += -D$(VECTOR_TABLE_LOCATION)
ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif

CDEFS += -D$(RUN_MODE) -D$(CHIP)
ADEFS += -D$(RUN_MODE) -D$(CHIP)


# Compiler flags.

ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
THUMB_IW = -mthumb-interwork
else 
THUMB    = 
THUMB_IW = 
endif

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW) 
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used 
#CFLAGS += -mapcs-frame 
#CFLAGS += -fomit-frame-pointer
CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += -fpromote-loop-indices
CFLAGS += -Wall -Wextra
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
#CFLAGS += -pedantic
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-*-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = 

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS  = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
CPLUSPLUS_LIB = -lstdc++
#CPLUSPLUS_LIB += -lsupc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
#not in CPP
#LDFLAGS += -nostartfiles
LDFLAGS += -lc
LDFLAGS += $(MATH_LIB)
LDFLAGS += -lc -lgcc
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS)) 

#LDFLAGS += -u TIM1_CC_IRQHandler -u ADC1_2_IRQHandler -u DMA1_Channel1_IRQHandler 

# Set linker-script name depending on selected run-mode and submodel name
ifeq ($(RUN_MODE),RAM_RUN)
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP)_ram.ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/sram.lds
else 
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP)_flash.ld
#LDFLAGS +=-T$(LINKERSCRIPTPATH)/stm32_flash.ld
#LDFLAGS +=-T$(LINKERSCRIPTPATH)/stm32f10x_flash_offset_1.ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/flash.lds
endif

# ---------------------------------------------------------------------------
# Options for OpenOCD flash-programming
# see openocd.pdf/openocd.texi for further information
#
OOCD_LOADFILE+=$(OUTDIR)/$(TARGET).elf
# if OpenOCD is in the $PATH just set OPENOCDEXE=openocd
OOCD_EXE=./OpenOCD/bin/openocd
# debug level
OOCD_CL=-d0
#OOCD_CL=-d3
# interface and board/target settings (using the OOCD target-library here)
## OOCD_CL+=-f interface/jtagkey2.cfg -f target/stm32.cfg
OOCD_CL+=-f interface/jtagkey.cfg -f target/stm32.cfg
# initialize
OOCD_CL+=-c init
# enable "fast mode" - can be disabled for tests
OOCD_CL+=-c "fast enable"
# show the targets
OOCD_CL+=-c targets
# commands to prepare flash-write
OOCD_CL+= -c "reset halt"
# increase JTAG frequency a little bit - can be disabled for tests
OOCD_CL+= -c "jtag_khz 1200"
# flash-write and -verify
OOCD_CL+=-c "flash write_image erase $(OOCD_LOADFILE)" -c "verify_image $(OOCD_LOADFILE)"
# reset target
OOCD_CL+=-c "reset run"
# terminate OOCD after programming
OOCD_CL+=-c shutdown
# ---------------------------------------------------------------------------

# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
REMOVE  = $(REMOVE_CMD) -f
SHELL   = sh
###COPY    = cp
ifneq ($(or $(COMSPEC), $(ComSpec)),)
$(info COMSPEC detected $(COMSPEC) $(ComSpec))
ifeq ($(findstring cygdrive,$(shell env)),)
SHELL:=$(or $(COMSPEC),$(ComSpec))
SHELL_IS_WIN32=1
else
$(info cygwin detected)
endif
endif
$(info SHELL is $(SHELL))

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = --------  begin, mode: $(RUN_MODE)  --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = ---- Linking :
MSG_COMPILING = ---- Compiling C :
MSG_COMPILING_ARM = ---- Compiling C ARM-only:
MSG_COMPILINGCPP = ---- Compiling C++ :
MSG_COMPILINGCPP_ARM = ---- Compiling C++ ARM-only:
MSG_ASSEMBLING = ---- Assembling:
MSG_ASSEMBLING_ARM = ---- Assembling ARM-only:
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

# Default target.
#all: begin gccversion sizebefore build sizeafter finished end
all: begin createdirs gccversion build sizeafter finished end

elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss 
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin


ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else 
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else 
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else 
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

# Create output directories.
ifdef SHELL_IS_WIN32
createdirs:
	-@md $(OUTDIR) >NUL 2>&1 || echo "" >NUL
	-@md $(OUTDIR)\dep >NUL 2>&1 || echo "" >NUL
else
createdirs:
	-@mkdir $(OUTDIR) 2>/dev/null || echo "" >/dev/null
	-@mkdir $(OUTDIR)/dep 2>/dev/null || echo "" >/dev/null
endif

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

finished:
##	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)

# Display sizes of sections.
ELFSIZE = $(SIZE) -A  $(OUTDIR)/$(TARGET).elf
##ELFSIZE = $(SIZE) --format=Berkeley --common $(OUTDIR)/$(TARGET).elf
sizebefore:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
#	@if [ -f  $(OUTDIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
#	@echo $(ALLOBJ)

# Program the device.
ifeq ($(FLASH_TOOL),UVISION)
# Program the device with Keil's uVision (needs configured uVision-workspace). 
program: $(OUTDIR)/$(TARGET).hex
	@echo "Programming with uVision"
	C:\Keil\uv3\Uv3.exe -f uvisionflash.Uv2 -ouvisionflash.txt
else
ifeq ($(FLASH_TOOL),OPENOCD)
# Program the device with Dominic Rath's OPENOCD in "batch-mode", needs cfg and "reset-script".
program: $(OUTDIR)/$(TARGET).elf
	@echo "Programming with OPENOCD"
ifdef SHELL_IS_WIN32 
	$(subst /,\,$(OOCD_EXE)) $(OOCD_CL)
else
	$(OOCD_EXE) $(OOCD_CL)
endif
else
# Program the device using lpc21isp (for NXP2k and ADuC UART bootloader)
program: $(OUTDIR)/$(TARGET).hex
	@echo $(MSG_LPC21_RESETREMINDER)
	-$(LPC21ISP) $(LPC21ISP_OPTIONS) $(LPC21ISP_FLASHFILE) $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)
endif
endif

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@
	
# Create final output file (.bin) from ELF output file.
%.bin: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
ifeq "$(strip $(CPPSRC)$(CPPARM))" ""
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ -nostartfiles $(LDFLAGS)
else
	$(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
endif


# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo "hallo"
	@echo $(MSG_ASSEMBLING) $$< to $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING_ARM) $$< to $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$< to $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING_ARM) $$< to $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP) $$< to $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP_ARM) $$< to $$@
	$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 


# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Target: clean project.
clean: begin clean_list finished end

clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)/$(TARGET).map
	$(REMOVE) $(OUTDIR)/$(TARGET).elf
	$(REMOVE) $(OUTDIR)/$(TARGET).hex
	$(REMOVE) $(OUTDIR)/$(TARGET).bin
	$(REMOVE) $(OUTDIR)/$(TARGET).sym
	$(REMOVE) $(OUTDIR)/$(TARGET).lss
	$(REMOVE) $(ALLOBJ)
	$(REMOVE) $(LSTFILES)
	$(REMOVE) $(DEPFILES)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(CPPSRC:.cpp=.s)
	$(REMOVE) $(CPPSRCARM:.cpp=.s)

## Create object files directory - now done if special make target
##$(shell mkdir $(OBJDIR) 2>/dev/null)

# Include the dependency files.
##-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)
-include $(wildcard dep/*)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program createdirs

