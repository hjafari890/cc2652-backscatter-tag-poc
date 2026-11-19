SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR ?= C:/ti/simplelink_cc13xx_cc26xx_sdk_8_33_00_16
XDC_INSTALL_DIR := C:/ti/xdctools_3_62_01_15_core
SYSCONFIG_TOOL := C:/ti/sysconfig_1.21.1/sysconfig_cli.bat
TICLANG_ARMCOMPILER := C:/ti/ccs2101/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS

include $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/imports.mak

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
LNK = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"

SYSCFG_DIR = syscfg_gen
SYSCFG_CMD = "$(SYSCONFIG_TOOL)" --compiler ticlang --product "$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json"

SYSCFG_GEN_C = $(SYSCFG_DIR)/ti_drivers_config.c $(SYSCFG_DIR)/ti_devices_config.c
SYSCFG_GEN_H = $(SYSCFG_DIR)/ti_drivers_config.h
SYSCFG_GEN_OPT = $(SYSCFG_DIR)/ti_utils_build_compiler.opt
SYSCFG_GEN_LNK = $(SYSCFG_DIR)/ti_utils_build_linker.cmd.genlibs

COMMON_CFLAGS = -I. -I$(SYSCFG_DIR) \
    @"$(SYSCFG_GEN_OPT)" \
    -Oz \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/nortos" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/nortos/posix" \
    -gdwarf-3 \
    -mcpu=cortex-m4 \
    -march=armv7e-m \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -Wall -Wno-unused-parameter

LFLAGS = "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/nortos" \
    "$(SYSCFG_GEN_LNK)" \
    cc13x2_cc26x2_nortos.cmd \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    "-L$(TICLANG_ARMCOMPILER)/lib" \
    -llibc.a

COMMON_OBJS = main_nortos.obj ti_drivers_config.obj ti_devices_config.obj

all: backscatter_poc_tag.out backscatter_poc_cw.out

$(SYSCFG_GEN_H) $(SYSCFG_GEN_C) $(SYSCFG_GEN_OPT) $(SYSCFG_GEN_LNK): backscatter_tag.syscfg
	@ echo Generating SysConfig files...
	@ if not exist "$(SYSCFG_DIR)" mkdir "$(SYSCFG_DIR)"
	@ $(SYSCFG_CMD) --output "$(SYSCFG_DIR)" backscatter_tag.syscfg

ti_drivers_config.obj: $(SYSCFG_DIR)/ti_drivers_config.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -c $< -o $@

ti_devices_config.obj: $(SYSCFG_DIR)/ti_devices_config.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -c $< -o $@

main_nortos.obj: main_nortos.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -c $< -o $@

backscatter_poc_tag.obj: backscatter_tag.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -DINITIAL_MODE=3 -c $< -o $@

backscatter_poc_cw.obj: backscatter_tag.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -DINITIAL_MODE=2 -c $< -o $@

backscatter_poc_off.obj: backscatter_tag.c $(SYSCFG_GEN_H)
	@ echo Building $@
	@ $(CC) $(COMMON_CFLAGS) -DINITIAL_MODE=0 -c $< -o $@

backscatter_poc_tag.out: backscatter_poc_tag.obj $(COMMON_OBJS)
	@ echo Linking $@
	@ $(LNK) -Wl,-u,_c_int00 backscatter_poc_tag.obj $(COMMON_OBJS) $(LFLAGS) "-Wl,-m,backscatter_poc_tag.map" -o $@

backscatter_poc_cw.out: backscatter_poc_cw.obj $(COMMON_OBJS)
	@ echo Linking $@
	@ $(LNK) -Wl,-u,_c_int00 backscatter_poc_cw.obj $(COMMON_OBJS) $(LFLAGS) "-Wl,-m,backscatter_poc_cw.map" -o $@

backscatter_poc_off.out: backscatter_poc_off.obj $(COMMON_OBJS)
	@ echo Linking $@
	@ $(LNK) -Wl,-u,_c_int00 backscatter_poc_off.obj $(COMMON_OBJS) $(LFLAGS) "-Wl,-m,backscatter_poc_off.map" -o $@

clean:
	@ echo Cleaning build artifacts...
	@ -del /Q *.obj *.out *.map 2>NUL
	@ -rmdir /S /Q "$(SYSCFG_DIR)" 2>NUL
