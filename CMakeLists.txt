set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# default GCC settings
set(TOOLCHAIN_PREFIX arm-none-eabi)
set(C_FLAGS "-fdata-sections -ffunction-sections -specs=nano.specs -Wl,--gc-sections")
set(CPP_FLAGS "-fno-rtti -fno-exceptions -fno-threadsafe-statics")

# compiler settings
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc ${C_FLAGS})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc ${C_FLAGS})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++ ${C_FLAGS} ${CPP_FLAGS})
set(OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)

set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

cmake_minimum_required(VERSION 3.12)

# compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(PROJ_PATH ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: " ${CMAKE_BUILD_TYPE})

# core project settings
project(F413-cm-test)
enable_language(C ASM)

# linker script
set(LINKER_SCRIPT "${PROJ_PATH}/STM32F413ZHTx_FLASH.ld")

# compiler options
set(CFLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wall -fdata-sections -ffunction-sections -g -gdwarf-2")
set(CMAKE_C_FLAGS ${CFLAGS})
set(CMAKE_CXX_FLAGS "${CFLAGS} -Og")
SET(CMAKE_ASM_FLAGS "${CFLAGS} -x assembler-with-cpp")
#set(CMAKE_GENERATOR Ninja)

# symbol definitions
add_compile_definitions(
    STM32F413xx
    USE_HAL_DRIVER
    _DEBUG
    UNICODE
    _UNICODE
)

# linker options
add_link_options(
    -T${LINKER_SCRIPT}
    -lc 
    -lm 
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segment
    -Wl,--print-memory-usage
)

# project source files
file(GLOB PROJECT_SOURCE_FILES CONFIGURE_DEPENDS Core/Src/*.c)

add_executable(
    ${CMAKE_PROJECT_NAME}
    ${PROJECT_SOURCE_FILES}
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)
# include directories
include_directories(
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

# compile and link libraries
add_library(
    CMSIS
    Core/Src/system_stm32f4xx.c
)

add_library(
    STARTUP
    startup_stm32f413xx.s
)

target_link_libraries(
    ${CMAKE_PROJECT_NAME}
    CMSIS
    STARTUP
)

# Execute post-build to print size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)

# save hex and bin files
set(HEX "${OBJCOPY} -O ihex")
set(BIN "${OBJCOPY} -O binary -S")

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${OBJCOPY} ARGS -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${OBJCOPY} ARGS -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)