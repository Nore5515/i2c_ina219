# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/noah/esp/esp-idf/components/bootloader/subproject"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix/tmp"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix/src"
  "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/noah/projects/gp/internal/battery-logger/i2c_self_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
