# Copyright (c) 2023 Spyrosoft Synergy S.A.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#!/usr/bin/env bash

ROS_DISTRO='iron'
FAST_BUILD=false
CLEAN_BUILD=false

POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--ros-distro)
      ROS_DISTRO="$2"
      shift
      shift
      ;;
    -f|--fast-build)
      FAST_BUILD=true
      shift 
      ;;
    -c|--clean-build)
      CLEAN_BUILD=true
      shift 
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift
      ;;
  esac
done


if [ "$CLEAN_BUILD" = true ]; then
    FAST_BUILD=false
fi

if [ "$FAST_BUILD" = false ]; then
    mkdir -p vendor/spyrosoft/roscar_tools/libmicroros/prebuilt/arm64/include/{rcl,rcutils,rmw,rosidl_runtime_c,rosidl_typesupport_interface,rcl_action,std_msgs,action_msgs,unique_identifier_msgs,builtin_interfaces,type_description_interfaces,service_msgs,rosidl_dynamic_typesupport}
    touch vendor/spyrosoft/roscar_tools/libmicroros/prebuilt/arm64/libmicroros.a
    source build/envsetup.sh
    lunch sdk_car_x86_64-userdebug
    make nothing
fi

./vendor/spyrosoft/roscar_tools/generate_cmake_toolchain.py
cp vendor/spyrosoft/roscar_tools/colcon.meta microros_ws/

source /opt/ros/$ROS_DISTRO/setup.bash

pushd microros_ws
    if [ "$CLEAN_BUILD" = true ]; then
        rm -rf build install log firmware
    fi

    if [ "$FAST_BUILD" = false ]; then
        sudo apt update && rosdep update
        rosdep install --from-paths src --ignore-src -y
        colcon build
    fi

    source install/local_setup.bash

    if ! [ -d "firmware" ]; then
        ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
        touch firmware/mcu_ws/ros2/libyaml_vendor/COLCON_IGNORE
    fi

    ros2 run micro_ros_setup build_firmware.sh $(pwd)/android_clang_toolchain.cmake $(pwd)/colcon.meta
popd # microros_ws

# Copy the prebuilt library
mkdir -p vendor/spyrosoft/roscar_tools/libmicroros/prebuilt/arm64
cp -r microros_ws/firmware/build/* vendor/spyrosoft/roscar_tools/libmicroros/prebuilt/arm64/
