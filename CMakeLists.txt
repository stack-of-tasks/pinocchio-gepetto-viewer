cmake_minimum_required(VERSION 3.2)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME pinocchio-gepetto-viewer)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL http://github.com/jmirabel/pinocchio-gepetto-viewer)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
set(DOXYGEN_USE_TEMPLATE_CSS ON)
set(DOXYGEN_INPUT "\"${CMAKE_SOURCE_DIR}/include\" \"${CMAKE_SOURCE_DIR}/src\" \"${CMAKE_SOURCE_DIR}/doc\"")
set(DOXYGEN_FILE_PATTERNS "*.cpp *.hpp")
set(DOXYGEN_MACRO_EXPANSION "YES")
set(CXX_DISABLE_WERROR ON)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Add a required dependency
add_project_dependency(gepetto-viewer)
add_project_dependency(gepetto-viewer-corba REQUIRED)
add_project_dependency(pinocchio REQUIRED)

# if(gepetto-viewer_FOUND)
#  CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)
#  add_subdirectory(plugins)
# endif()

set(${PROJECT_NAME}_HEADERS
  include/pinocchio/gepetto/viewer.hpp
  include/pinocchio/gepetto/viewer.hxx)

add_library(pinocchio-gepetto-viewer SHARED
  ${${PROJECT_NAME}_HEADERS}
  src/viewer.cc)
target_link_libraries(pinocchio-gepetto-viewer PUBLIC pinocchio::pinocchio gepetto-viewer-corba::gepetto-viewer-corba)

add_executable(ur5 tests/ur5.cc)
target_link_libraries(ur5 PRIVATE pinocchio-gepetto-viewer)
add_test(NAME ur5 COMMAND ur5)

install(TARGETS pinocchio-gepetto-viewer
        EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION lib)
