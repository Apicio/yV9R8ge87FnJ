cmake_minimum_required(VERSION 2.8)
project(naoregnition)
find_package(qibuild)

qi_create_bin(naoregnition main.cpp)
qi_use_lib(naoregnition OPENCV2_CORE OPENCV2_HIGHGUI ALCOMMON ALPROXIES ALVISION)
