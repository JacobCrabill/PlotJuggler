
include_directories(../)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC  datastream_zmq.ui  )

SET( SRC datastream_zmq.cpp )

add_library(DataStreamZMQ SHARED ${SRC} ${UI_SRC}  )

target_link_libraries(DataStreamZMQ
    ${Qt5Widgets_LIBRARIES}
    plotjuggler_base
    zmq
    )

install(TARGETS DataStreamZMQ DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )


