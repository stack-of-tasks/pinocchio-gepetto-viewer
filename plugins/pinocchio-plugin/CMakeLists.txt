find_package (Qt5 REQUIRED COMPONENTS Core Widgets Gui)
foreach (component "Core" "Widgets" "Gui")
  list(APPEND QT_INCLUDE_DIRS ${Qt5${component}_INCLUDE_DIRS})
  list(APPEND QT_LIBRARIES    ${Qt5${component}_LIBRARIES}   )
endforeach (component "Core" "Widgets" "Gui")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake-module/)

FINDPYTHON()
find_package(PythonQt REQUIRED COMPONENTS QtAll REQUIRED)

if(DEFINED PINOCCHIO_WITH_PYTHON_INTERFACE)
  search_for_boost_python(REQUIRED)
  set(MOC_OPTIONS -DPINOCCHIO_WITH_PYTHON_INTERFACE)
endif()


GEPETTO_GUI_PLUGIN(pinocchio-plugin
  HEADERS
  plugin.hh
  frame-tree.hh
  python.hh

  HEADERS_NO_MOC
  header-view.hh

  MOC_OPTIONS ${MOC_OPTIONS}

  FORMS
  frame-tree.ui

  SOURCES
  plugin.cc
  frame-tree.cc
  python.cc

  LINK_DEPENDENCIES
  pinocchio-gepetto-viewer
  ${PYTHON_LIBRARY}
  ${PYTHONQT_LIBRARIES}
  )

target_include_directories(pinocchio-plugin PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(pinocchio-plugin SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS} ${PYTHONQT_INCLUDE_DIR})

if(DEFINED PINOCCHIO_WITH_PYTHON_INTERFACE)
  target_compile_options(pinocchio-plugin PRIVATE -DPINOCCHIO_WITH_PYTHON_INTERFACE)
  target_link_boost_python(pinocchio-plugin PRIVATE)
endif()
