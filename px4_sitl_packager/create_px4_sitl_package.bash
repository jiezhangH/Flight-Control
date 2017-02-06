#!/usr/bin/env bash

# Simple script to build the PX4 firmware for SITL and zip artefacts to be used.
#
# @author: Julian Oes <julian@oes.ch>

# This script needs to run inside the firmware directory.
if [ ! -f ./Firmware.sublime-project ]; then
    echo "Please run inside Firmware directory"
    exit 1
fi

# This creates a version such as "0.1.0-1-g533aeb".
version=`git describe --always --tags`

tmpdir="/tmp/sitl"
builddir="build_posix_sitl_ekf2"

if [ ! -f "$builddir/src/firmware/posix/px4" ]; then
    echo "Please run 'make posix_sitl_ekf2 gazebo_typhoon_h480' manually first"
    exit 1
fi


mkdir -p $tmpdir/posix-configs/SITL/init/ekf2
mkdir -p $tmpdir/Tools/sitl_gazebo/worlds
mkdir -p $tmpdir/Tools/sitl_gazebo/models
mkdir -p $tmpdir/ROMFS/px4fmu_common/mixers
mkdir -p $tmpdir/src/firmware/posix
mkdir -p $tmpdir/build_gazebo

#cp -r $builddir/* $tmpdir
cp $builddir/src/firmware/posix/px4 $tmpdir/src/firmware/posix/
cp $builddir/build_gazebo/libgazebo_geotagged_images_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libgazebo_lidar_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libgazebo_opticalFlow_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libLiftDragPlugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libmav_msgs.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_controller_interface.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_gimbal_controller_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_imu_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_mavlink_interface.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_motor_model.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_multirotor_base_plugin.dylib $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_wind_plugin.dylib $tmpdir/build_gazebo/
cp Tools/sitl_run.sh $tmpdir/Tools/
cp posix-configs/SITL/init/ekf2/typhoon_h480 $tmpdir/posix-configs/SITL/init/ekf2/
cp Tools/posix_lldbinit $tmpdir/Tools/
cp Tools/posix.gdbinit $tmpdir/Tools/
cp Tools/setup_gazebo.bash $tmpdir/Tools/
cp Tools/sitl_gazebo/worlds/typhoon_h480.world $tmpdir/Tools/sitl_gazebo/worlds/
cp -r Tools/sitl_gazebo/models/typhoon_h480 $tmpdir/Tools/sitl_gazebo/models
cp ROMFS/px4fmu_common/mixers/hexa_x.main.mix $tmpdir/ROMFS/px4fmu_common/mixers/
cp ROMFS/px4fmu_common/mixers/mount_legs.aux.mix $tmpdir/ROMFS/px4fmu_common/mixers/

# Create a bash script to start the sim
echo "#!/bin/bash -e

if [ -L ROMFS ]; then
    rm ROMFS
fi
if [ -L test_data ]; then
    rm test_data
fi
Tools/sitl_run.sh src/firmware/posix/px4 posix-configs/SITL/init/ekf2 none gazebo typhoon_h480 . ." > \
    $tmpdir/typhoon_sitl.bash
chmod +x $tmpdir/typhoon_sitl.bash

# copy everything into zip
curdir=`pwd`
(cd $tmpdir && zip -r $curdir/Yuneec-SITL-flightcode-macOS-$version.zip .)
