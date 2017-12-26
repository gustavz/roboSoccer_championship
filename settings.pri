INCLUDEPATH += /DIST/lehre/lab_roso/tech/usr/include/rtdb  /DIST/lehre/lab_roso/tech/usr/include/roboctrl /DIST/lehre/lab_roso/tech/usr/include/libmath
QMAKE_CXXFLAGS += -isystem/DIST/lehre/lab_roso/tech/usr/include/rtdb -isystem/DIST/lehre/lab_roso/tech/usr/include/roboctrl -isystem/DIST/lehre/lab_roso/tech/usr/include/libmath
QMAKE_CXXFLAGS += -O2 -Wall -Wextra -Werror -Wuninitialized

LIBS += -L/DIST/lehre/lab_roso/tech/usr/lib \        
        #-lVimbaC \
        -lpololu_rtdb \
        -lkogmo_rtdb \
        -lboost_system\
        #-lmath

LIBS += -L/usr/lib -L/DIST/lehre/lab_roso/tech/usr/lib -lpololu_rtdb -lkogmo_rtdb -lQtGui -lQtCore -lpthread

#INCLUDEPATH += /usr/include/boost
