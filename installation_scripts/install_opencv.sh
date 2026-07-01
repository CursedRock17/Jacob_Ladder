#!/bin/bash
#
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#
# Modified from this script: https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.6.0_Jetson.sh
# Do not use U.S. mirror as your APT source, it has missing files.

version="${OPENCV_VERSION:-4.10.0}"
folder="${OPENCV_SRC_DIR:-$HOME/opencv_src}"
install_prefix="${OPENCV_INSTALL_PREFIX:-/usr/local}"
build_list="${OPENCV_BUILD_LIST:-core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,objdetect,aruco}"
build_python="${OPENCV_BUILD_PYTHON:-OFF}"
with_cuda="${OPENCV_WITH_CUDA:-ON}"
with_cudnn="${OPENCV_WITH_CUDNN:-ON}"
default_build_jobs="$(($(nproc) - 1))"
if [ "${default_build_jobs}" -lt 1 ]; then
    default_build_jobs=1
fi
build_jobs="${OPENCV_BUILD_JOBS:-${default_build_jobs}}"
remove_default_opencv="${REMOVE_DEFAULT_OPENCV:-ask}"

set -e

if [ "${remove_default_opencv}" = "ask" ]; then
    for (( ; ; ))
    do
        echo "Do you want to remove the default OpenCV (yes/no)?"
        read rm_old

        if [ "$rm_old" = "yes" ]; then
            remove_default_opencv="yes"
            break
        elif [ "$rm_old" = "no" ]; then
            remove_default_opencv="no"
            break
        fi
    done
fi

if [ "${remove_default_opencv}" = "yes" ]; then
        echo "** Remove other OpenCV first"
        sudo apt -y purge *libopencv*
fi


echo "------------------------------------"
echo "** Install requirement (1/4)"
echo "------------------------------------"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2
sudo apt-get install -y curl


echo "------------------------------------"
echo "** Download opencv "${version}" (2/4)"
echo "------------------------------------"
mkdir -p "$folder"
cd "${folder}"
if [ ! -d "opencv-${version}" ]; then
    curl -fL https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
    unzip -q opencv-${version}.zip
    rm opencv-${version}.zip
fi
if [ ! -d "opencv_contrib-${version}" ]; then
    curl -fL https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
    unzip -q opencv_contrib-${version}.zip
    rm opencv_contrib-${version}.zip
fi
cd "opencv-${version}/"


echo "------------------------------------"
echo "** Build opencv "${version}" (3/4)"
echo "------------------------------------"
if [ "${OPENCV_CLEAN_BUILD:-0}" = "1" ]; then
    rm -rf release
fi
mkdir -p release
cd release/
# CUDA_ARCH_BIN 8.7 is AGX orin
cmake -D WITH_CUDA="${with_cuda}" -D WITH_CUDNN="${with_cudnn}" -D CUDA_ARCH_BIN="7.2,8.7" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules -D BUILD_LIST="${build_list}" -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3="${build_python}" -D BUILD_JAVA=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX="${install_prefix}" ..
# save a core
make -j"${build_jobs}"

echo "------------------------------------"
echo "** Install opencv "${version}" (4/4)"
echo "------------------------------------"
if [ -w "${install_prefix}" ]; then
    make install
else
    sudo make install
fi
if ! grep -q "LD_LIBRARY_PATH=${install_prefix}/lib" ~/.bashrc; then
    echo "export LD_LIBRARY_PATH=${install_prefix}/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
fi
source ~/.bashrc


echo "** Install opencv "${version}" successfully"
echo "** Bye :)"
