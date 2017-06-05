#To make a project qtable, just add a .pro file with the following tags
#run with  qmake -o Makefile trabalho1.pro to generate the makefile
#run make

OBJECTS_DIR  = generated_files #Intermediate object files directory
MOC_DIR      = generated_files #Intermediate moc files directory
INCLUDEPATH  = include /opt/ros/indigo/include
DEPENDPATH   = include

#TARGET  = main

QMAKE_CXX = ccache g++

CONFIG -= app_bundle

HEADERS += include/tr1/*.h

SOURCES += src/obstacle_avoidance.cpp \
           src/obstacle_avoidance_node.cpp

#LIBS    += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lboost_timer -lboost_system

QMAKE_LFLAGS += -c
