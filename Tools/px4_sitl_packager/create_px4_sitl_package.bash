#!/usr/bin/env bash

# Simple script to build the PX4 firmware for SITL and zip artefacts to be used.
#
# @author: Julian Oes <julian@oes.ch>

# Stop on error
set -e


argc=$#
if [ $argc -ne 1 ];
then
	echo "Usage: create_px4_sitl_package.bash px4_firmware_path"
	exit 1
fi

firmwaredir=$1

# Check the Firmware path
if [ ! -f "$firmwaredir/Firmware.sublime-project" ]; then
    echo "Please run inside Firmware directory"
    exit 1
fi

# This creates a version such as "v1.6.1-131-ge7bd40aa84".
version=`cd $firmwaredir && git describe --always --tags --abbrev=10`

tmpdir="/tmp/sitl"
builddir="$firmwaredir/build_posix_sitl_default"

# Force gazebo to shutdown after building SITL
echo 'shutdown' >> $firmwaredir/posix-configs/SITL/init/ekf2/typhoon_h480

# Build SITL
cd $firmwaredir
HEADLESS=1 make posix_sitl_default gazebo_typhoon_h480

# Revert recent change
git checkout HEAD -- $firmwaredir/posix-configs/SITL/init/ekf2/typhoon_h480

if [ ! -f "$builddir/src/firmware/posix/px4" ]; then
    echo "Build 'make posix_sitl_default gazebo_typhoon_h480' failed. Exit."
    exit 1
fi

if [ "$(uname)" == "Darwin" ]; then
    lib_ending="dylib"
    os_string="macOS"
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    lib_ending="so"
    os_string="Linux"
fi

mkdir -p $tmpdir/posix-configs/SITL/init/default
mkdir -p $tmpdir/Tools/sitl_gazebo/worlds
mkdir -p $tmpdir/Tools/sitl_gazebo/models
mkdir -p $tmpdir/ROMFS/px4fmu_common/mixers
mkdir -p $tmpdir/src/firmware/posix
mkdir -p $tmpdir/build_gazebo

if [ "$(uname)" == "Darwin" ]; then
    # We need to remove the absolute paths that are generated for libmav_msgs.dylib
    builddir_abs=`(cd $builddir && pwd)`
    echo "$builddir_abs/build_gazebo/libmav_msgs.dylib"
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/libgazebo_geotagged_images_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/libgazebo_lidar_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/libgazebo_opticalFlow_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/libLiftDragPlugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/libmav_msgs.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_controller_interface.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_gimbal_controller_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_imu_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_mavlink_interface.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_motor_model.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_multirotor_base_plugin.$lib_ending
    install_name_tool -change "$builddir_abs/build_gazebo/libmav_msgs.dylib" "./build_gazebo/libmav_msgs.dylib" $builddir/build_gazebo/librotors_gazebo_wind_plugin.$lib_ending
fi


#cp -r $builddir/* $tmpdir
cp $builddir/src/firmware/posix/px4 $tmpdir/src/firmware/posix/

cp $builddir/build_gazebo/libgazebo_geotagged_images_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libgazebo_lidar_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libgazebo_opticalFlow_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libLiftDragPlugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/libmav_msgs.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_controller_interface.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_gimbal_controller_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_imu_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_mavlink_interface.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_motor_model.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_multirotor_base_plugin.$lib_ending $tmpdir/build_gazebo/
cp $builddir/build_gazebo/librotors_gazebo_wind_plugin.$lib_ending $tmpdir/build_gazebo/
cp $firmwaredir/Tools/sitl_run.sh $tmpdir/Tools/
cp $firmwaredir/posix-configs/SITL/init/ekf2/typhoon_h480 $tmpdir/posix-configs/SITL/init/default/
cp $firmwaredir/Tools/posix_lldbinit $tmpdir/Tools/
cp $firmwaredir/Tools/posix.gdbinit $tmpdir/Tools/
cp $firmwaredir/Tools/setup_gazebo.bash $tmpdir/Tools/
cp $firmwaredir/Tools/sitl_gazebo/worlds/typhoon_h480.world $tmpdir/Tools/sitl_gazebo/worlds/
cp -r $firmwaredir/Tools/sitl_gazebo/models/typhoon_h480 $tmpdir/Tools/sitl_gazebo/models
cp $firmwaredir/ROMFS/px4fmu_common/mixers/hexa_x.main.mix $tmpdir/ROMFS/px4fmu_common/mixers/
cp $firmwaredir/ROMFS/px4fmu_common/mixers/mount_legs.aux.mix $tmpdir/ROMFS/px4fmu_common/mixers/

# Add bash script to start it
cp $firmwaredir/Tools/px4_sitl_packager/typhoon_sitl.bash $tmpdir/typhoon_sitl.bash
chmod +x $tmpdir/typhoon_sitl.bash
# And add a readme
cp $firmwaredir/Tools/px4_sitl_packager/README_package.md $tmpdir/README.md

# copy everything into zip
curdir=`pwd`
(cd $tmpdir && zip -r $curdir/Yuneec-SITL-Simulation-$os_string-$version.zip .)
