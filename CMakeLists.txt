cmake_minimum_required(VERSION 3.16)
project(mfr3duo_mujoco VERSION 0.1.0 LANGUAGES CXX)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "mfr3duo_mujoco supports Linux only.")
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mfr3duo_description REQUIRED)
find_package(romujoco CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED src/config.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC romujoco::romujoco)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mfr3duo_sim src/main.cpp)
target_compile_features(mfr3duo_sim PRIVATE cxx_std_17)
target_link_libraries(mfr3duo_sim PRIVATE ${PROJECT_NAME})
target_compile_options(mfr3duo_sim PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS ${PROJECT_NAME} mfr3duo_sim
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp)

if(BUILD_TESTING)
  include(CTest)

  add_executable(${PROJECT_NAME}_config_test test/config_test.cpp)
  target_compile_features(${PROJECT_NAME}_config_test PRIVATE cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_config_test PRIVATE ${PROJECT_NAME})
  add_test(NAME ${PROJECT_NAME}.config COMMAND ${PROJECT_NAME}_config_test)

  add_executable(${PROJECT_NAME}_simulation_smoke_test test/simulation_smoke_test.cpp)
  target_compile_features(${PROJECT_NAME}_simulation_smoke_test PRIVATE cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_simulation_smoke_test PRIVATE ${PROJECT_NAME})
  add_test(NAME ${PROJECT_NAME}.simulation_smoke COMMAND ${PROJECT_NAME}_simulation_smoke_test)
endif()

ament_package(CONFIG_EXTRAS "cmake/romujoco_dependency.cmake")
