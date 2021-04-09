CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

# These variables have to be defined before running SETUP_PROJECT
SET(PROJECT_NAME pinocchio-gepetto-viewer)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL http://github.com/jmirabel/pinocchio-gepetto-viewer)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
SET(DOXYGEN_USE_TEMPLATE_CSS ON)
SET(DOXYGEN_INPUT "\"${CMAKE_SOURCE_DIR}/include\" \"${CMAKE_SOURCE_DIR}/src\" \"${CMAKE_SOURCE_DIR}/doc\"")
SET(DOXYGEN_FILE_PATTERNS "*.cpp *.hpp")
SET(DOXYGEN_MACRO_EXPANSION "YES")
SET(CXX_DISABLE_WERROR ON)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

OPTION(BUILD_TESTS "Build tests" ON)
OPTION(BUILD_PLUGINS "Build plugins" OFF)

# Add a required dependency
ADD_PROJECT_DEPENDENCY(gepetto-viewer)
ADD_PROJECT_DEPENDENCY(gepetto-viewer-corba REQUIRED)
ADD_PROJECT_DEPENDENCY(pinocchio REQUIRED)

IF(BUILD_PLUGINS)
  IF(gepetto-viewer_FOUND)
   CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)
   ADD_SUBDIRECTORY(plugins)
  ENDIF()
ENDIF()

SET(
  ${PROJECT_NAME}_HEADERS
  ${PROJECT_SOURCE_DIR}/include/pinocchio/gepetto/viewer.hpp
  ${PROJECT_SOURCE_DIR}/include/pinocchio/gepetto/viewer.hxx
)

ADD_LIBRARY(
  ${PROJECT_NAME} 
  SHARED
  ${${PROJECT_NAME}_HEADERS}
  ${PROJECT_SOURCE_DIR}/src/viewer.cc
)
TARGET_LINK_LIBRARIES(
  ${PROJECT_NAME}
  PUBLIC 
  pinocchio::pinocchio 
  gepetto-viewer::gepetto-viewer
  gepetto-viewer-corba::gepetto-viewer-corba
)
TARGET_INCLUDE_DIRECTORIES(
  ${PROJECT_NAME} 
  PUBLIC 
  $<INSTALL_INTERFACE:include>
)
IF(PINOCCHIO_USE_HPP_FCL)
  target_compile_definitions(
    ${PROJECT_NAME} 
    PRIVATE
    PINOCCHIO_WITH_HPP_FCL
  )
ENDIF()

IF(BUILD_TESTS)
  ADD_EXECUTABLE(ur5 tests/ur5.cc)
  TARGET_LINK_LIBRARIES(ur5 PRIVATE pinocchio-gepetto-viewer)
  ADD_TEST(NAME ur5 COMMAND ur5)
ENDIF()

INSTALL(TARGETS pinocchio-gepetto-viewer
        EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION lib)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})