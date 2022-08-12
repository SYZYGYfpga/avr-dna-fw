cmake_minimum_required(VERSION 3.15)

set(AVR_MCU attiny44 CACHE STRING "MCU")
set(AVR_L_FUSE 0xE2 CACHE STRING "Low Fuse")
set(AVR_H_FUSE 0xDD CACHE STRING "High Fuse")
set(AVR_E_FUSE 0xFE CACHE STRING "Extended Fuse")
set(AVRDUDE_PROGRAMMER avrispmkII CACHE STRING "Programmer")
set(AVRDUDE_PORT usb CACHE STRING "Port of programmer")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)
set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_OBJDUMP avr-objdump)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_ASM_FLAGS "")

set(SHARED_LIBS OFF)
set(STATIC_LIBS ON)

set(COMMON_FLAGS "${COMMON_FLAGS} -mmcu=${AVR_MCU} -fno-builtin -ffunction-sections -fdata-sections -fomit-frame-pointer -MMD -MP -Os")
string(APPEND CMAKE_C_FLAGS " ${COMMON_FLAGS}")
string(APPEND CMAKE_ASM_FLAGS " ${COMMON_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS "-mmcu=${AVR_MCU} -Wl,--gc-sections -mrelax")

project(avr-dna-fw LANGUAGES C)

add_executable(${CMAKE_PROJECT_NAME}.elf
  ${PROJECT_SOURCE_DIR}/src/main.c
  ${PROJECT_SOURCE_DIR}/src/USI_TWI_Slave.c
  ${PROJECT_SOURCE_DIR}/src/syzygy_dna_fw.c
  ${PROJECT_SOURCE_DIR}/src/syzygy_helpers.c
  ${PROJECT_SOURCE_DIR}/src/syzygy_seq.c
  )

target_compile_features(${CMAKE_PROJECT_NAME}.elf
  PRIVATE
    c_std_99
  )

target_include_directories(${CMAKE_PROJECT_NAME}.elf
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

add_custom_command(
  OUTPUT ${CMAKE_PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -j .text -j .data -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
  DEPENDS ${CMAKE_PROJECT_NAME}.elf
)

add_custom_command(
  OUTPUT ${CMAKE_PROJECT_NAME}.eep
  COMMAND ${CMAKE_OBJCOPY} -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.eep
  DEPENDS ${CMAKE_PROJECT_NAME}.elf
)

add_custom_target(
  size
  avr-size -C --mcu=${AVR_MCU} ${CMAKE_PROJECT_NAME}.elf
  DEPENDS ${CMAKE_PROJECT_NAME}.elf
  )

find_program(AVRDUDE avrdude)

if (AVRDUDE)
  add_custom_target(
    flash
    ${AVRDUDE} -p ${AVR_MCU} -c ${AVRDUDE_PROGRAMMER} -P ${AVRDUDE_PORT} -U flash:w:${CMAKE_PROJECT_NAME}.hex
    DEPENDS ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Programming flash"
  )

  add_custom_target(
    eeprom
    ${AVRDUDE} -p ${AVR_MCU} -c ${AVRDUDE_PROGRAMMER} -P ${AVRDUDE_PORT} -U eeprom:w:${CMAKE_PROJECT_NAME}.eep
    DEPENDS ${CMAKE_PROJECT_NAME}.eep
    COMMENT "Programming eeprom"
   )

  add_custom_target(
    fuse
    ${AVRDUDE} -p ${AVR_MCU} -c ${AVRDUDE_PROGRAMMER} -P ${AVRDUDE_PORT} -U lfuse:w:${AVR_L_FUSE}:m -U hfuse:w:${AVR_H_FUSE}:m -U efuse:w:${AVR_E_FUSE}:m
    COMMENT "Programming fuse"
   )
endif()
