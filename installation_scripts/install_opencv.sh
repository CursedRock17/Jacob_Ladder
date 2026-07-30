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
# cudev/cuda* are what actually give you cv::cuda::GpuMat and cv2.cuda.*; without
# them WITH_CUDA only accelerates bits inside the core modules. dnn is needed for
# cv2.dnn (and its CUDA backend). aruco is a contrib module -- the OpenCV that
# ships with the JetPack/ARK image does not have it, which is why
# aruco_tracker's `find_package(OpenCV 4 REQUIRED COMPONENTS ... aruco)` fails.
# BUILD_LIST is a whitelist: any module not named here (or pulled in as a
# dependency of one that is) gets "Disabled by dependency", regardless of the
# corresponding BUILD_opencv_* flag. python3 in particular must be listed or
# BUILD_opencv_python3=ON has no effect and you get no cv2 bindings.
build_list="${OPENCV_BUILD_LIST:-core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,objdetect,dnn,aruco,python3,cudev,cudaarithm,cudawarping,cudaimgproc,cudafeatures2d}"
build_python="${OPENCV_BUILD_PYTHON:-ON}"
with_cuda="${OPENCV_WITH_CUDA:-ON}"
# cuDNN only matters for cv2.dnn's CUDA backend. JetPack 6.2 ships cuDNN 9, which
# older OpenCV releases cannot detect -- if cmake errors on cuDNN, re-run with
# OPENCV_WITH_CUDNN=OFF. Nothing in this repo needs it (YOLO runs through torch).
with_cudnn="${OPENCV_WITH_CUDNN:-ON}"
# Orin NX / Orin Nano / AGX Orin are all sm_87. Xavier is 7.2. Listing only the
# arch you have roughly halves the build time.
cuda_arch="${OPENCV_CUDA_ARCH:-8.7}"
default_build_jobs="$(($(nproc) - 1))"
if [ "${default_build_jobs}" -lt 1 ]; then
    default_build_jobs=1
fi
build_jobs="${OPENCV_BUILD_JOBS:-${default_build_jobs}}"
# Default to "no": the ROS Humble debs (cv_bridge, image_geometry, image_pipeline)
# link libopencv_*.so.4.5d from the Ubuntu packages, so `apt purge *libopencv*`
# takes a chunk of the ROS install with it.
remove_default_opencv="${REMOVE_DEFAULT_OPENCV:-ask}"

# uv venv to install the cv2 python bindings into. Empty => the default
# ${install_prefix}/lib/python3.X/site-packages, which a venv will not see.
python_venv="${OPENCV_PYTHON_VENV:-${VIRTUAL_ENV:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.venv}}"

set -e

# nvcc is not on PATH by default on JetPack; without it cmake either fails or
# quietly builds with CUDA disabled, which is the whole point of this script.
if ! command -v nvcc >/dev/null 2>&1; then
    for cuda_home in /usr/local/cuda /usr/local/cuda-12; do
        if [ -x "${cuda_home}/bin/nvcc" ]; then
            export PATH="${cuda_home}/bin:${PATH}"
            export CUDACXX="${cuda_home}/bin/nvcc"
            break
        fi
    done
fi
if [ "${with_cuda}" = "ON" ] && ! command -v nvcc >/dev/null 2>&1; then
    echo "ERROR: WITH_CUDA=ON but nvcc was not found. Install cuda-toolkit or set" >&2
    echo "       PATH to include it, or re-run with OPENCV_WITH_CUDA=OFF." >&2
    exit 1
fi

if [ "${remove_default_opencv}" = "ask" ]; then
    for (( ; ; ))
    do
        echo "Do you want to remove the default OpenCV (yes/no)?"
        echo "  Say 'no' unless you know what you are doing: the ROS Humble debs"
        echo "  depend on the Ubuntu libopencv 4.5 packages and will be removed too."
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
# libtbb2 (2020.3) conflicts with libtbb-dev (2021.5) on jammy -- install only the dev pkg.
sudo apt-get install -y libtbb-dev libjpeg-dev libpng-dev libtiff-dev
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
cmake_args=(
    -D CMAKE_BUILD_TYPE=RELEASE
    -D CMAKE_INSTALL_PREFIX="${install_prefix}"
    -D WITH_CUDA="${with_cuda}"
    -D WITH_CUDNN="${with_cudnn}"
    -D OPENCV_DNN_CUDA="${with_cudnn}"
    -D CUDA_ARCH_BIN="${cuda_arch}"
    -D CUDA_ARCH_PTX=""
    -D OPENCV_GENERATE_PKGCONFIG=ON
    -D OPENCV_EXTRA_MODULES_PATH="../../opencv_contrib-${version}/modules"
    -D BUILD_LIST="${build_list}"
    -D WITH_GSTREAMER=ON
    -D WITH_LIBV4L=ON
    -D BUILD_opencv_python3="${build_python}"
    -D BUILD_JAVA=OFF
    -D BUILD_TESTS=OFF
    -D BUILD_PERF_TESTS=OFF
    -D BUILD_EXAMPLES=OFF
)

# Point the python bindings at the project venv, otherwise cv2 lands in
# ${install_prefix}/lib/python3.X/site-packages where the venv cannot see it.
if [ "${build_python}" = "ON" ] && [ -x "${python_venv}/bin/python" ]; then
    venv_site_packages="$("${python_venv}/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
    # A venv has no libpython/headers of its own, and OpenCV's detection does not
    # follow the venv back to the base interpreter -- it reports
    # "Python 3: Libraries: NO" and silently drops the bindings module. Point it
    # at the base interpreter's library and headers explicitly.
    venv_py_include="$("${python_venv}/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["include"])')"
    venv_py_library="$("${python_venv}/bin/python" -c 'import os, sysconfig; print(os.path.join(sysconfig.get_config_var("LIBDIR"), sysconfig.get_config_var("LDLIBRARY")))')"
    venv_numpy_include="$("${python_venv}/bin/python" -c 'import numpy; print(numpy.get_include())' 2>/dev/null || true)"
    if [ ! -f "${venv_py_library}" ] || [ ! -f "${venv_py_include}/Python.h" ]; then
        echo "ERROR: python development files not found (${venv_py_library}," >&2
        echo "       ${venv_py_include}/Python.h). Install them with:" >&2
        echo "         sudo apt-get install -y python3-dev" >&2
        exit 1
    fi
    echo "** Installing cv2 python bindings into ${venv_site_packages}"
    cmake_args+=(
        -D PYTHON3_EXECUTABLE="${python_venv}/bin/python"
        -D PYTHON3_INCLUDE_DIR="${venv_py_include}"
        -D PYTHON3_LIBRARY="${venv_py_library}"
        -D OPENCV_PYTHON3_INSTALL_PATH="${venv_site_packages}"
    )
    if [ -n "${venv_numpy_include}" ]; then
        cmake_args+=(-D PYTHON3_NUMPY_INCLUDE_DIRS="${venv_numpy_include}")
    fi
    if "${python_venv}/bin/python" -c 'import cv2' >/dev/null 2>&1; then
        echo "** WARNING: a cv2 is already importable in ${python_venv} (probably the"
        echo "**          opencv-python wheel). It will shadow this CUDA build."
        echo "**          Remove it with: uv pip uninstall opencv-python"
    fi
fi

cmake "${cmake_args[@]}" ..
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
# Register the prefix with the dynamic linker instead of leaning on
# LD_LIBRARY_PATH, which does not propagate to systemd units or ros2 launch.
if [ "${install_prefix}" != "/usr" ] && [ "${install_prefix}" != "/usr/local" ]; then
    echo "${install_prefix}/lib" | sudo tee /etc/ld.so.conf.d/opencv-cuda.conf >/dev/null
fi
sudo ldconfig


echo "------------------------------------"
echo "** Verify"
echo "------------------------------------"
echo "pkg-config opencv4: $(pkg-config --modversion opencv4 2>&1)"
verify_failed=0
if [ "${build_python}" = "ON" ] && [ -x "${python_venv}/bin/python" ]; then
    OPENCV_EXPECT_VERSION="${version}" "${python_venv}/bin/python" - <<'PY' || verify_failed=1
import os
import sys

try:
    import cv2
except ImportError as exc:
    sys.exit(f"FAIL: cannot import cv2: {exc}")

expected = os.environ["OPENCV_EXPECT_VERSION"]
print("cv2", cv2.__version__, "from", cv2.__file__)
ok = True

if cv2.__version__ != expected:
    print(f"FAIL: expected cv2 {expected}, got {cv2.__version__}.")
    print("      Something else on sys.path is shadowing this build -- most")
    print("      likely the opencv-python pip wheel or the system python3-opencv")
    print("      deb. Remove the wheel with: uv pip uninstall opencv-python")
    ok = False

devices = cv2.cuda.getCudaEnabledDeviceCount()
print("CUDA devices:", devices)
if devices < 1:
    print("FAIL: cv2 reports no CUDA devices -- these bindings are not CUDA-enabled.")
    ok = False

sys.exit(0 if ok else 1)
PY
fi
if [ "${verify_failed}" != "0" ]; then
    echo ""
    echo "** VERIFICATION FAILED -- see the messages above. The C++ libraries in"
    echo "** ${install_prefix} may still be fine; it is the python bindings that are wrong."
    exit 1
fi

echo "** Install opencv ${version} successfully"
echo "** Note: colcon builds should be told which OpenCV to use, e.g."
echo "**   colcon build --cmake-args -DOpenCV_DIR=${install_prefix}/lib/cmake/opencv4"
echo "** Bye :)"
