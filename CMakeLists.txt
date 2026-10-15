cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Solar_Light_Tracking_System)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        lvgl/lv_port_lcd_stm32.h
        lvgl/lv_port_indev.h
        lvgl/lv_conf.h
        lvgl/lv_version.h
        lvgl/lvgl.h
        lvgl/lvgl_private.h
        Core/interface/FT6336.c
        Core/interface/AS5600.c
        Core/interface/AS5600.h
        Core/interface/SimpleFOC_Motor.c
        Core/interface/SimpleFOC_Motor.h
        Core/interface/vofa.c
        Core/interface/vofa.h
        Middlewares/ST/ARM/DSP/Inc/arm_math.h
        Core/common/com_Debug.c
        Core/common/com_Debug.h)

# 只编译当前屏幕、触摸和 user_home 页面实际需要的 LVGL 模块。
# 不再递归编译 demos、测试、PC 平台驱动以及未使用的控件，减少 Flash 占用。
file(GLOB LVGL_CORE_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/core/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/stdlib/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/stdlib/builtin/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/draw/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/draw/sw/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/draw/sw/blend/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/draw/convert/lv_draw_buf_convert.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/display/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/fs/lv_fs.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/indev/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/image/lv_bin_decoder.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/image/lv_image_decoder.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/font/lv_font.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/font/lv_font_montserrat_16.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/font/fmt_txt/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/cache/lv_cache.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/cache/lv_cache_entry.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/cache/class/lv_cache_lru_rb.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/cache/instance/lv_image_cache.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/misc/cache/instance/lv_image_header_cache.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/osal/lv_os.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/osal/lv_freertos.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/tick/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/themes/lv_theme.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/themes/default/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/layouts/lv_layout.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/layouts/flex/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/others/translation/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/bar/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/button/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/dropdown/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/label/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/slider/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/widgets/tabview/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/drivers/display/mipi/lv_lcd_generic_mipi.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/drivers/display/mipi/lv_st7789.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/lvgl/src/lv_init.c"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

#添加单个.c文件或者遍历文件夹的.c文件
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/App/UiTask.c
        Core/App/DebugTask.c
        Core/App/MotorTask.c
        Core/App/SmaplingTask.c
        Core/common/filter.c
        Core/common/pid.c
        Core/ui/user_home.c
        Core/ui/noto_sans_sc_16.c
        lvgl/porting/lv_port_lcd_stm32.c
        lvgl/porting/lv_port_indev.c
        ${LVGL_CORE_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/lvgl
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/ui
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/interface
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/App
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/common
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Types
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

)

# VOFA FireWater 使用 snprintf 格式化浮点数。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
