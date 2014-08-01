# This file is a makefile included from the top level makefile which
# defines the sources built for the target.

# Define the prefix to this directory. 
# Note: The name must be unique within this build and should be
#       based on the root of the project
TARGET_STDPERIPH_PATH = STM32F4xx_StdPeriph_Driver
TARGET_STDPERIPH_SRC_PATH = $(TARGET_STDPERIPH_PATH)/src

# Add tropicssl include to all objects built for this target
INCLUDE_DIRS += $(TARGET_STDPERIPH_PATH)/inc

# C source files included in this build.
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/misc.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_adc.c
#CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_bkp.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_crc.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_dma.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_exti.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_flash.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_gpio.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_i2c.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_iwdg.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_pwr.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_rcc.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_rtc.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_spi.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_tim.c
CSRC += $(TARGET_STDPERIPH_SRC_PATH)/stm32f4xx_usart.c

# C++ source files included in this build.
CPPSRC +=

# ASM source files included in this build.
ASRC +=

