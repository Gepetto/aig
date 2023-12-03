cmake_minimum_required(VERSION 3.10)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME aig)
set(PROJECT_DESCRIPTION "Analytical inverse geomery for MPC in bipeds")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Instal *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()

# #################################################### Check if the submodule
# cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
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
# ##############################################################################

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Dependencies
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(eiquadprog REQUIRED)
add_project_dependency(example-robot-data 4.0.7 REQUIRED)

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif()

if(BUILD_PYTHON_INTERFACE)
  find_package(eigenpy 2.7.10 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

# Main library
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
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio
                                             eiquadprog::eiquadprog)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  set(${PY_NAME}_SOURCES python/contact_6d.cpp python/aig_modules.cpp
                         python/dyna_com.cpp python/biped_ig.cpp)
  add_library(${PY_NAME}_py SHARED ${${PY_NAME}_SOURCES})
  target_link_libraries(${PY_NAME}_py PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
  get_relative_rpath(${${PY_NAME}_INSTALL_DIR} ${PY_NAME}_INSTALL_RPATH)
  set_target_properties(
    ${PY_NAME}_py
    PROPERTIES INSTALL_RPATH "${${PY_NAME}_INSTALL_RPATH}"
               LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
               OUTPUT_NAME ${PY_NAME}
               PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX})
  target_link_boost_python(${PY_NAME}_py PRIVATE)
  install(TARGETS ${PY_NAME}_py DESTINATION ${${PY_NAME}_INSTALL_DIR})
  install(DIRECTORY python/aig DESTINATION ${${PY_NAME}_INSTALL_DIR}/..)
endif()

if(BUILD_TESTING)
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
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif()
