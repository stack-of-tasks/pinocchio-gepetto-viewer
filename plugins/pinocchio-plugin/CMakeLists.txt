find_package (Qt5 REQUIRED COMPONENTS Core Widgets Gui)
FOREACH (component "Core" "Widgets" "Gui")
  LIST(APPEND QT_INCLUDE_DIRS ${Qt5${component}_INCLUDE_DIRS})
  LIST(APPEND QT_LIBRARIES    ${Qt5${component}_LIBRARIES}   )
ENDFOREACH (component "Core" "Widgets" "Gui")

GEPETTO_GUI_PLUGIN(pinocchio-plugin
  HEADERS
  plugin.hh
  frame-tree.hh

  FORMS
  frame-tree.ui

  SOURCES
  plugin.cc
  frame-tree.cc

  LINK_DEPENDENCIES
  "pinocchio-gepetto-viewer"
  )

target_include_directories(pinocchio-plugin PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
