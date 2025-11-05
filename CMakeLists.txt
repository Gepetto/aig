cmake_minimum_required(VERSION 3.22.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME aig)
set(PROJECT_DESCRIPTION "Analytical inverse geomery for MPC in bipeds")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()

# Check if the submodule cmake have been initialized
if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(STATUS "JRL cmakemodules found in the source tree.")
  set(JRL_CMAKE_MODULES "${CMAKE_SOURCE_DIR}/cmake")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

add_project_dependency(pinocchio REQUIRED)
add_project_dependency(eiquadprog REQUIRED)

if(BUILD_TESTING)
  add_project_dependency(example-robot-data REQUIRED)
  add_project_dependency(Boost REQUIRED COMPONENTS unit_test_framework)
endif()

# Main library
if(BUILD_STANDALONE_PYTHON_INTERFACE)
  add_project_dependency(${PROJECT_NAME} REQUIRED CONFIG)
else()
  set(${PROJECT_NAME}_HEADERS
      include/${PROJECT_NAME}/arm_ig.hpp #
      include/${PROJECT_NAME}/leg_ig.hpp #
      include/${PROJECT_NAME}/biped_ig.hpp #
      include/${PROJECT_NAME}/dyna_com.hpp #
      include/${PROJECT_NAME}/python.hpp #
      include/${PROJECT_NAME}/contact6d.hpp #
  )
  set(${PROJECT_NAME}_SOURCES
      src/arm_ig.cpp #
      src/leg_ig.cpp #
      src/biped_ig.cpp #
      src/dyna_com.cpp #
      src/contac6d.cpp #
  )
  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                     ${${PROJECT_NAME}_HEADERS})
  add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
  target_include_directories(${PROJECT_NAME}
                             PUBLIC $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio
                                               eiquadprog::eiquadprog)

  if(SUFFIX_SO_VERSION)
    set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif()
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  set(${PY_NAME}_SOURCES python/contact_6d.cpp python/aig_modules.cpp
                         python/dyna_com.cpp python/biped_ig.cpp)
  add_library(${PY_NAME}_py SHARED ${${PY_NAME}_SOURCES})
  target_link_libraries(${PY_NAME}_py PUBLIC ${PROJECT_NAME}::${PROJECT_NAME}
                                             eigenpy::eigenpy)
  get_relative_rpath(${${PY_NAME}_INSTALL_DIR} ${PY_NAME}_INSTALL_RPATH)
  set_target_properties(
    ${PY_NAME}_py
    PROPERTIES INSTALL_RPATH "${${PY_NAME}_INSTALL_RPATH}"
               LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
               OUTPUT_NAME ${PY_NAME}
               PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX})
  install(TARGETS ${PY_NAME}_py DESTINATION ${${PY_NAME}_INSTALL_DIR})
  install(DIRECTORY python/aig DESTINATION ${${PY_NAME}_INSTALL_DIR}/..)
endif()

if(BUILD_TESTING AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  # Unit-Tests dependencies
  set(${PROJECT_NAME}_CPP_TESTS
      test_dependencies_usage #
      test_leg_ig #
      test_arm_ig #
      test_biped_ig #
  )

  foreach(unittest_name ${${PROJECT_NAME}_CPP_TESTS})
    add_unit_test(${unittest_name} tests/${unittest_name}.cpp)
    target_link_libraries(
      ${unittest_name} ${PROJECT_NAME} Boost::unit_test_framework
      example-robot-data::example-robot-data)
    target_compile_definitions(${unittest_name}
                               PUBLIC "-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN")
  endforeach()
endif()

# Installation
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif()
