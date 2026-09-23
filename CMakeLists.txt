cmake_minimum_required(VERSION 3.16)
project(mfr3duo_mujoco VERSION 0.2.0 LANGUAGES CXX)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "mfr3duo_mujoco supports Linux only.")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(romujoco CONFIG REQUIRED)

find_path(
  MFR3DUO_DESCRIPTION_SHARE_DIR
  NAMES mjcf/scene.xml
  HINTS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES share/mfr3duo_description mfr3duo_description
  DOC "Directory containing the installed MFR3Duo mjcf directory")

add_library(${PROJECT_NAME} SHARED
  src/config.cpp
  src/simulation.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} PRIVATE romujoco::romujoco)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

if(MFR3DUO_DESCRIPTION_SHARE_DIR)
  target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE
      MFR3DUO_MUJOCO_DEFAULT_DESCRIPTION_SHARE_DIR="${MFR3DUO_DESCRIPTION_SHARE_DIR}")
endif()

add_executable(mfr3duo_sim src/main.cpp)
target_compile_features(mfr3duo_sim PRIVATE cxx_std_17)
target_link_libraries(mfr3duo_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
target_compile_options(mfr3duo_sim PRIVATE -Wall -Wextra -Wpedantic)

option(MFR3DUO_MUJOCO_BUILD_EXAMPLES "Build mfr3duo_mujoco examples" OFF)
if(MFR3DUO_MUJOCO_BUILD_EXAMPLES)
  add_executable(mfr3duo_control_example examples/control.cpp)
  target_compile_features(mfr3duo_control_example PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_control_example PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
  target_compile_options(mfr3duo_control_example PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(${PROJECT_NAME}_config_test test/config_test.cpp)
  target_compile_features(${PROJECT_NAME}_config_test PRIVATE cxx_std_17)
  target_include_directories(
    ${PROJECT_NAME}_config_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(
    ${PROJECT_NAME}_config_test
    PRIVATE
      ${PROJECT_NAME}::${PROJECT_NAME}
      romujoco::romujoco)
  add_test(NAME ${PROJECT_NAME}.config COMMAND ${PROJECT_NAME}_config_test)

  add_executable(${PROJECT_NAME}_simulation_smoke_test test/simulation_smoke_test.cpp)
  target_compile_features(${PROJECT_NAME}_simulation_smoke_test PRIVATE cxx_std_17)
  target_link_libraries(
    ${PROJECT_NAME}_simulation_smoke_test PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
  add_test(
    NAME ${PROJECT_NAME}.simulation_smoke
    COMMAND ${PROJECT_NAME}_simulation_smoke_test)
endif()

configure_package_config_file(
  cmake/mfr3duo_mujocoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mfr3duo_mujocoConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/mfr3duo_mujocoConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(
  TARGETS ${PROJECT_NAME} mfr3duo_sim
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY docs DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/mfr3duo_mujocoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/mfr3duo_mujocoConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})