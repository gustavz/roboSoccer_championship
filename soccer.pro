TEMPLATE = app
TARGET = soccer
#choose between the following for boost support
include(settings.pri)
#include(settingsSim.pri)


DEPENDPATH += src
INCLUDEPATH += src

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core

CONFIG +=  debug


# Input
HEADERS += \
    lib/GoalKeeper.h \
    lib/Agent.h \
    lib/FieldPlayer.h \
    lib/Physics.h \
    lib/GameControl.h \
    lib/Enemy.h \
    lib/RunnableObject.h\
    lib/StateMachine.h \
    lib/Path.h \
    lib/Geometry.h \
    lib/Debug.h \
    helper/Vector2d.h \
    helper/Line.h \
    helper/LineSegment.h \
    helper/Obstacle.h \
    helper/Circle.h \
    helper/Quadrangle.h \
    helper/Node.h \
    helper/KdTree.h \
    helper/Trajectory.h \
    lib/Ball.h
SOURCES += \
    src/GoalKeeper.cpp \
    src/main.cpp \
    src/GameControl.cpp \
    src/FieldPlayer.cpp \
    src/Physics.cpp \
    src/Agent.cpp \
    src/Path.cpp \
    src/Debug.cpp \
    helper/Vector2d.cpp \
    helper/Circle.cpp \
    helper/Line.cpp \
    helper/LineSegment.cpp \
    helper/Quadrangle.cpp \
    helper/Obstacle.cpp \
    helper/Node.cpp \
    helper/KdTree.cpp \
    helper/Trajectory.cpp\
    src/Enemy.cpp

QMAKE_CXX=g++
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_TARGETS += dox
QMAKE_CXXFLAGS += -Wno-unused-variable
QMAKE_CXXFLAGS += -Wno-unused-but-set-variable -std=gnu++11 -fpermissive

FORMS +=











































































