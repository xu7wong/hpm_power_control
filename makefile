OUT ?= app
BUILD_DIR = build
CHIP_NAME = HPM6280
FLASH_SIE = 4M

SRCS = \
app/src/main.c \
app/adc_trigger.c \
bsp/board.c \
soc/$(CHIP_NAME)/boot/hpm_bootheader.c \
soc/$(CHIP_NAME)/toolchains/reset.c \
soc/$(CHIP_NAME)/toolchains/trap.c \
soc/$(CHIP_NAME)/hpm_l1c_drv.c \
soc/$(CHIP_NAME)/system.c \
soc/$(CHIP_NAME)/hpm_sysctl_drv.c \
soc/$(CHIP_NAME)/hpm_clock_drv.c \
drivers/src/hpm_pllctlv2_drv.c \
drivers/src/hpm_gpio_drv.c \
drivers/src/hpm_uart_drv.c \
drivers/src/hpm_adc16_drv.c \
drivers/src/hpm_gptmr_drv.c \
drivers/src/hpm_pwm_drv.c \
drivers/src/hpm_pmp_drv.c \
drivers/src/hpm_dma_drv.c

ASM_SRCS = \
soc/$(CHIP_NAME)/toolchains/gcc/start.S

OBJS = $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))
vpath %.c $(sort $(dir $(SRCS)))

OBJS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SRCS:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SRCS)))

#LDSCRIPT = soc/$(CHIP_NAME)/toolchains/gcc/ram.ld
LDSCRIPT = soc/$(CHIP_NAME)/toolchains/gcc/flash_xip.ld

DEFINES = \
-DCONFIG_HAS_HPMSDK_ACMP=y \
-DCONFIG_HAS_HPMSDK_ADC16=y \
-DCONFIG_HAS_HPMSDK_CRC=y \
-DCONFIG_HAS_HPMSDK_DAC=y \
-DCONFIG_HAS_HPMSDK_DMA=y \
-DCONFIG_HAS_HPMSDK_GPIO=y \
-DCONFIG_HAS_HPMSDK_GPTMR=y \
-DCONFIG_HAS_HPMSDK_I2C=y \
-DCONFIG_HAS_HPMSDK_LIN=y \
-DCONFIG_HAS_HPMSDK_MCAN=y \
-DCONFIG_HAS_HPMSDK_MCHTMR=y \
-DCONFIG_HAS_HPMSDK_PCFG=y \
-DCONFIG_HAS_HPMSDK_PLA=y \
-DCONFIG_HAS_HPMSDK_PLLCTLV2=y \
-DCONFIG_HAS_HPMSDK_PMP=y \
-DCONFIG_HAS_HPMSDK_PMU=y \
-DCONFIG_HAS_HPMSDK_PTPC=y \
-DCONFIG_HAS_HPMSDK_PWM=y \
-DCONFIG_HAS_HPMSDK_QEI=y \
-DCONFIG_HAS_HPMSDK_RNG=y \
-DCONFIG_HAS_HPMSDK_RTC=y \
-DCONFIG_HAS_HPMSDK_SDM=y \
-DCONFIG_HAS_HPMSDK_SDP=y \
-DCONFIG_HAS_HPMSDK_SPI=y \
-DCONFIG_HAS_HPMSDK_TSNS=y \
-DCONFIG_HAS_HPMSDK_UART=y \
-DCONFIG_HAS_HPMSDK_USB=y \
-DCONFIG_HAS_HPMSDK_WDG=y \
-DFLASH_XIP=1

CFLAGS = \
 -Wall \
 -Wundef \
 -Wno-format \
 -fomit-frame-pointer \
 -fno-builtin \
 -ffunction-sections \
 -fdata-sections \
 -mabi=ilp32 \
 -march=rv32imac \
 -g

LDFLAGS = \
-Wl,--no-whole-archive \
-static \
-nostartfiles \
-Wl,--gc-sections \
-Wl,--print-memory-usage \
-mabi=ilp32 \
-march=rv32imac \
--specs=nano.specs \
-u _printf_float \
-u _scanf_float \
-Xlinker --defsym=_flash_size=$(FLASH_SIE) \
-T $(LDSCRIPT)

LIBS =

CC = riscv32-unknown-elf-gcc
CP = riscv32-unknown-elf-objcopy
AS = riscv32-unknown-elf-gcc -x assembler-with-cpp
# AS = riscv32-unknown-elf-as

INCLUDES = -I . \
	-I /home/carl/hpm/riscv32-unknown-elf-newlib-multilib/riscv32-unknown-elf/include \
	-I /home/carl/hpm/riscv32-unknown-elf-newlib-multilib/lib/gcc/riscv32-unknown-elf/11.1.0/include \
	-I ./arch \
	-I ./soc/$(CHIP_NAME) \
	-I ./soc/$(CHIP_NAME)/boot \
	-I ./soc/$(CHIP_NAME)/toolchains \
	-I ./soc/ip \
	-I ./drivers/inc \
	-I ./bsp \
	-I ./app/inc

all: $(BUILD_DIR) $(OUT)

$(OUT): $(OBJS)
	$(CC) $(LDFLAGS) $(LIBS) $^ -o $(BUILD_DIR)/$(OUT).elf
	$(CP) -O binary -S $(BUILD_DIR)/$(OUT).elf $(BUILD_DIR)/$(OUT).bin

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c
	$(CC) $(DEFINES) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.S
	$(AS) $(DEFINES) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
