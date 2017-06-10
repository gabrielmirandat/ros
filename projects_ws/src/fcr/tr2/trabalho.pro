#To make a project qtable, just add a .pro file with the following tags
#run with  qmake -o Makefile trabalho.pro to generate the makefile
#run make

OBJECTS_DIR= generated_files #Intermediate object files directory
MOC_DIR    = generated_files #Intermediate moc files directory
INCLUDEPATH = include /opt/ros/indigo/include
DEPENDPATH = include

#TARGET  = main

#QMAKE_CXX = ccache g++

CONFIG -= app_bundle

HEADERS +=  include/tr2/graph.h \
            include/tr2/obstacle_avoidance.h \
            include/tr2/point_kinematics.h \
            include/tr2/topological_map.h \
            include/tr2/occupancy_grid.h \
            include/tr2/ns_node.h \
            include/tr2/trabalho.h


SOURCES +=  src/graph.cpp \
            src/node.cpp \
            src/obstacle_avoidance.cpp \
            src/point_kinematics.cpp \
            src/topological_map.cpp \
            src/occupancy_grid.cpp \
            src/trabalho.cpp


#LIBS    += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lboost_timer -lboost_system

QMAKE_LFLAGS += -c
