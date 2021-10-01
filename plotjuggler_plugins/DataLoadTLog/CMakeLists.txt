
include_directories(  ../ )
include_directories(  ../../plotjuggler_app/ )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC
    ../selectlistdialog.ui
    )

SET( SRC
    dataload_tlog.cpp
    )

add_library(DataLoadTLog SHARED ${SRC} ${UI_SRC}  )

target_link_libraries(DataLoadTLog
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    plotjuggler_base)

install(TARGETS DataLoadTLog DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )

