# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/fvalde/esp/esp-idf/components/bootloader/subproject"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/tmp"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/src"
  "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/fvalde/projects/noxwork/rfm95rtos/examples/rfm95rtos-tx/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
