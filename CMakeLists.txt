cmake_minimum_required(VERSION 3.2)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME pinocchio-gepetto-viewer)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL http://github.com/jmirabel/pinocchio-gepetto-viewer)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Add a required dependency
add_project_dependency(gepetto-viewer-corba REQUIRED)
add_project_dependency(pinocchio REQUIRED)

set(${PROJECT_NAME}_HEADERS
  include/pinocchio/gepetto/viewer.hh)

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
