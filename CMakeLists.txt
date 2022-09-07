cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME aig)
set(PROJECT_DESCRIPTION "Analytical inverse geomery for MPC in bipeds")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})

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
# ##############################################################################

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(aig ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

# Dependencies
add_project_dependency(pinocchio REQUIRED)
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  find_package(example-robot-data REQUIRED)
endif()

# Main library
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/arm_ig.hpp #
    include/${PROJECT_NAME}/leg_ig.hpp #
    include/${PROJECT_NAME}/biped_ig.hpp #
)
set(${PROJECT_NAME}_SOURCES
    src/arm_ig.cpp #
    src/leg_ig.cpp #
    src/biped_ig.cpp #
)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
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
    target_link_libraries(${unittest_name} ${PROJECT_NAME}
      Boost::unit_test_framework example-robot-data::example-robot-data)
    target_compile_definitions(${unittest_name} PUBLIC
      "-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN")
  endforeach()
endif()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
