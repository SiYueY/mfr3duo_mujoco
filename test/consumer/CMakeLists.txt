cmake_minimum_required(VERSION 3.22)
project(mfr3duo_mujoco_consumer LANGUAGES CXX)

find_package(mfr3duo_mujoco CONFIG REQUIRED)

add_executable(mfr3duo_mujoco_consumer main.cpp)
target_compile_features(mfr3duo_mujoco_consumer PRIVATE cxx_std_17)
target_link_libraries(
  mfr3duo_mujoco_consumer
  PRIVATE mfr3duo_mujoco::mfr3duo_mujoco)
