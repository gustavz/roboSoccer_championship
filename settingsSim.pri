INCLUDEPATH += /DIST/lehre/lab_roso/tech/usr_sim/include/rtdb /DIST/lehre/lab_roso/tech/usr_sim/include/roboctrl /DIST/lehre/lab_roso/tech/usr_sim/include/libmath
QMAKE_CXXFLAGS += -isystem/DIST/lehre/lab_roso/tech/usr_sim/include/rtdb -isystem/DIST/lehre/lab_roso/tech/usr_sim/include/roboctrl  -isystem/DIST/lehre/lab_roso/tech/usr_sim/include/libmath
QMAKE_CXXFLAGS += -O2 -Wall -Wextra -Werror -Wuninitialized

LIBS += -L/DIST/lehre/lab_roso/tech/usr_sim/lib \
        -lpololu_rtdb \
        -lkogmo_rtdb \
        -lboost_system\
#        -lmath \
