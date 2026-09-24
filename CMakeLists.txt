cmake_minimum_required(VERSION 3.22)
project(mfr3duo_mujoco VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "mfr3duo_mujoco supports Linux only.")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(romujoco 0.1.0 CONFIG REQUIRED)

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
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  BUILD_RPATH_USE_ORIGIN ON
  INSTALL_RPATH "$ORIGIN")

if(MFR3DUO_DESCRIPTION_SHARE_DIR)
  target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE
      MFR3DUO_MUJOCO_DEFAULT_DESCRIPTION_SHARE_DIR="${MFR3DUO_DESCRIPTION_SHARE_DIR}")
endif()

add_executable(mfr3duo_sim tools/simulator/main.cpp)
target_compile_features(mfr3duo_sim PRIVATE cxx_std_17)
target_link_libraries(mfr3duo_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
target_compile_options(mfr3duo_sim PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(mfr3duo_sim PROPERTIES
  BUILD_RPATH_USE_ORIGIN ON
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

option(MFR3DUO_MUJOCO_BUILD_TELEOP "Build the keyboard teleoperation tool" ON)
set(
  MFR3DUO_MUJOCO_PINOCCHIO_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/install"
  CACHE PATH
  "Private Pinocchio 4.1.0 installation prefix used by mfr3duo_teleop")

function(mfr3duo_find_private_pinocchio)
  if(NOT EXISTS "${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX}")
    message(
      FATAL_ERROR
      "Private Pinocchio 4.1.0 is not installed.\n"
      "Run: ./scripts/pinocchio.sh install\n"
      "Or disable teleop with -DMFR3DUO_MUJOCO_BUILD_TELEOP=OFF.")
  endif()

  set(CMAKE_PREFIX_PATH
      "${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX};${CMAKE_PREFIX_PATH}")
  find_package(
    pinocchio 4.1.0 EXACT CONFIG REQUIRED
    PATHS "${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX}"
    NO_DEFAULT_PATH)

  if(NOT TARGET pinocchio::pinocchio_parsers)
    message(
      FATAL_ERROR
      "The private Pinocchio installation does not provide URDF parser support.")
  endif()
endfunction()

function(mfr3duo_configure_pinocchio_runtime target)
  target_link_directories(
    ${target}
    PRIVATE "${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX}/lib")
  set_property(
    TARGET ${target}
    APPEND PROPERTY BUILD_RPATH "${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX}/lib")
endfunction()

if(MFR3DUO_MUJOCO_BUILD_TELEOP)
  mfr3duo_find_private_pinocchio()

  add_executable(
    mfr3duo_teleop
    tools/teleop/main.cpp
    tools/teleop/keyboard.cpp
    tools/teleop/kinematics.cpp
    tools/teleop/teleop.cpp)
  target_compile_features(mfr3duo_teleop PRIVATE cxx_std_17)
  target_include_directories(
    mfr3duo_teleop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools/teleop)
  target_link_libraries(
    mfr3duo_teleop
    PRIVATE
      ${PROJECT_NAME}::${PROJECT_NAME}
      pinocchio::pinocchio
      pinocchio::pinocchio_parsers)
  target_compile_options(mfr3duo_teleop PRIVATE -Wall -Wextra -Wpedantic)
  mfr3duo_configure_pinocchio_runtime(mfr3duo_teleop)
  set_target_properties(mfr3duo_teleop PROPERTIES
    BUILD_RPATH_USE_ORIGIN ON
    INSTALL_RPATH
      "$ORIGIN/../${CMAKE_INSTALL_LIBDIR};${MFR3DUO_MUJOCO_PINOCCHIO_PREFIX}/lib")
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

  if(MFR3DUO_DESCRIPTION_SHARE_DIR)
    add_executable(${PROJECT_NAME}_simulation_smoke_test test/simulation_smoke_test.cpp)
    target_compile_features(${PROJECT_NAME}_simulation_smoke_test PRIVATE cxx_std_17)
    target_link_libraries(
      ${PROJECT_NAME}_simulation_smoke_test PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
    add_test(
      NAME ${PROJECT_NAME}.simulation_smoke
      COMMAND ${PROJECT_NAME}_simulation_smoke_test)
    set_tests_properties(
      ${PROJECT_NAME}.simulation_smoke
      PROPERTIES ENVIRONMENT "MFR3DUO_DESCRIPTION_PATH=${MFR3DUO_DESCRIPTION_SHARE_DIR}")

    if(MFR3DUO_MUJOCO_BUILD_TELEOP)
      add_executable(
        ${PROJECT_NAME}_kinematics_test
        test/kinematics_test.cpp
        tools/teleop/kinematics.cpp)
      target_compile_features(${PROJECT_NAME}_kinematics_test PRIVATE cxx_std_17)
      target_include_directories(
        ${PROJECT_NAME}_kinematics_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools/teleop)
      target_compile_definitions(
        ${PROJECT_NAME}_kinematics_test
        PRIVATE
          MFR3DUO_TEST_URDF_PATH="${MFR3DUO_DESCRIPTION_SHARE_DIR}/urdf/mfr3duo.urdf")
      target_link_libraries(
        ${PROJECT_NAME}_kinematics_test
        PRIVATE
          ${PROJECT_NAME}::${PROJECT_NAME}
          pinocchio::pinocchio
          pinocchio::pinocchio_parsers)
      mfr3duo_configure_pinocchio_runtime(${PROJECT_NAME}_kinematics_test)
      add_test(NAME ${PROJECT_NAME}.kinematics COMMAND ${PROJECT_NAME}_kinematics_test)

      add_executable(
        ${PROJECT_NAME}_teleop_smoke_test
        test/teleop_smoke_test.cpp
        tools/teleop/kinematics.cpp
        tools/teleop/teleop.cpp)
      target_compile_features(${PROJECT_NAME}_teleop_smoke_test PRIVATE cxx_std_17)
      target_include_directories(
        ${PROJECT_NAME}_teleop_smoke_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools/teleop)
      target_compile_definitions(
        ${PROJECT_NAME}_teleop_smoke_test
        PRIVATE
          MFR3DUO_TEST_SCENE_PATH="${MFR3DUO_DESCRIPTION_SHARE_DIR}/mjcf/scene.xml"
          MFR3DUO_TEST_URDF_PATH="${MFR3DUO_DESCRIPTION_SHARE_DIR}/urdf/mfr3duo.urdf")
      target_link_libraries(
        ${PROJECT_NAME}_teleop_smoke_test
        PRIVATE
          ${PROJECT_NAME}::${PROJECT_NAME}
          pinocchio::pinocchio
          pinocchio::pinocchio_parsers)
      mfr3duo_configure_pinocchio_runtime(${PROJECT_NAME}_teleop_smoke_test)
      add_test(NAME ${PROJECT_NAME}.teleop_smoke COMMAND ${PROJECT_NAME}_teleop_smoke_test)
    endif()
  else()
    message(STATUS "MFR3Duo model not found: runtime model tests are disabled.")
  endif()
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

if(MFR3DUO_MUJOCO_BUILD_TELEOP)
  install(TARGETS mfr3duo_teleop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

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