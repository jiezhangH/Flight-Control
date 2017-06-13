#!/bin/bash -e

if [ -L ROMFS ]; then
    rm ROMFS
fi
if [ -L test_data ]; then
    rm test_data
fi
Tools/sitl_run.sh src/firmware/posix/px4 posix-configs/SITL/init/default none gazebo typhoon_h480 . .
