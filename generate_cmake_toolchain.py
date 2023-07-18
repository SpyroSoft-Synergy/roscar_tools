#!/usr/bin/env python3

import re
import sys
import os
import json
import logging
from datetime import datetime

dir_path = os.path.dirname(os.path.realpath(__file__)) + '/'

def get_makefile_path():
    with open('out/soong/soong.variables', 'r') as soong_vars:
        vars = json.load(soong_vars)
        make_suffix = vars['Make_suffix']
        if make_suffix is None:
            logging.error('Could not obtain the make suffix')
            sys.exit(1)
        else:
            return os.path.abspath(f'out/soong/make_vars{make_suffix}.mk')

def parse_makefile(file_path):
    # Define a dictionary to store the variables
    variables = {}

    # Define a regular expression to match variable assignments
    variable_assignment_regex = re.compile(r'^([a-zA-Z0-9_]+)\s*:=\s*(.*)$')

    # Open the Makefile and read the lines
    with open(file_path, 'r') as makefile:
        for line in makefile:
            # Remove comments
            line = line.split('#')[0].strip()

            # Match the line against the regular expression
            match = variable_assignment_regex.match(line)
            if match:
                # If the line matches, store the variable in the dictionary
                variables[match.group(1)] = match.group(2)

    return variables

def find_in(key, input_string):
    start = input_string.find(key)
    if start == -1:
        return None
    start += len(key)
    end = input_string.find(' ', start)
    if end == -1:
        end = len(input_string)
    return input_string[start:end]


root_path = os.path.abspath('./') + '/'
makefile_path = get_makefile_path()
mk_vars = parse_makefile(makefile_path)

toolchain_file_content = f"""#
# This is an auto-generated toolchain file. Do not edit it manually.
# Generated on {datetime.now()}
#

# Without that flag CMake is not able to pass test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_SYSTEM_VERSION 31)

set(CMAKE_SYSROOT                         {root_path + find_in('--sysroot ', mk_vars['SOONG_CLANG_HOST_GLOBAL_CFLAGS'])})
set(CMAKE_ANDROID_STANDALONE_TOOLCHAIN    {root_path + find_in('--gcc-toolchain=', mk_vars['SOONG_CLANG_HOST_GLOBAL_CFLAGS'])})
set(TOOLCHAIN_TRIPLE                      {mk_vars['SOONG_CLANG_CONFIG_arm64_TARGET_TRIPLE']})

# specify the cross compiler
set(CMAKE_ASM_COMPILER                    {root_path + mk_vars['SOONG_CLANG']} CACHE STRING "")
SET(CMAKE_ASM_COMPILER_TARGET             ${{TOOLCHAIN_TRIPLE}})
set(CMAKE_C_COMPILER                      {root_path + mk_vars['SOONG_CLANG']} CACHE STRING "")
set(CMAKE_C_COMPILER_TARGET               ${{TOOLCHAIN_TRIPLE}})
set(CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN   ${{CMAKE_ANDROID_STANDALONE_TOOLCHAIN}})
set(CMAKE_CXX_COMPILER                    {root_path + mk_vars['SOONG_CLANG_CXX']} CACHE STRING "")
set(CMAKE_CXX_COMPILER_TARGET             ${{TOOLCHAIN_TRIPLE}})
set(CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN ${{CMAKE_ANDROID_STANDALONE_TOOLCHAIN}})


# specify the linker
set(CMAKE_C_LINK_EXECUTABLE     {root_path + mk_vars['SOONG_TARGET_LD']} CACHE STRING "")
set(CMAKE_CXX_LINK_EXECUTABLE   {root_path + mk_vars['SOONG_TARGET_LD']} CACHE STRING "")
set(CMAKE_LINKER                {root_path + mk_vars['SOONG_TARGET_LD']} CACHE STRING "")

set(CMAKE_C_FLAGS_INIT          "{mk_vars['SOONG_CLANG_TARGET_GLOBAL_CFLAGS']}" CACHE STRING "")
set(CMAKE_CXX_FLAGS_INIT        "{mk_vars['SOONG_CLANG_TARGET_GLOBAL_CPPFLAGS']}" CACHE STRING "")

set(CMAKE_C_FLAGS_DEBUG         "-O0 -g" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE       "-Os -DNDEBUG" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG       "${{CMAKE_C_FLAGS_DEBUG}}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE     "${{CMAKE_C_FLAGS_RELEASE}}" CACHE INTERNAL "")

set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
    {root_path + "bionic/libc/include"}
    {root_path + "bionic/libc/kernel/uapi"}
    {root_path + "bionic/libc/kernel/android/scsi"}
    {root_path + "bionic/libc/kernel/android/uapi"}
    {root_path + "bionic/libc/kernel/uapi/asm-arm64"}
    {root_path + "external/libcxx/include"}
)

set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
    ${{CMAKE_C_STANDARD_INCLUDE_DIRECTORIES}}
    ${{CMAKE_SYSROOT}}/usr/include/x86_64-linux-gnu/
    {root_path + '/synergycar/aosp/external/libcxx/include'}
)

add_compile_options(-fPIC)
add_compile_definitions(__ANDROID_API__=31 __ANDROID__)
add_link_options({mk_vars['SOONG_CLANG_TARGET_GLOBAL_LLDFLAGS']})
link_directories({root_path + "prebuilts/runtime/mainline/runtime/sdk/android/arm/lib/"})

# don't search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# search for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
"""

# Write the CMake toolchain file
with open('microros_ws/android_clang_toolchain.cmake', 'w') as toolchain_file:
    toolchain_file.write(toolchain_file_content)

print("CMake toolchain file has been saved to microros_ws/android_clang_toolchain.cmake")

