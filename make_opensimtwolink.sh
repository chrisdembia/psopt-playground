g++ -o opensimtwolink opensimtwolink.cpp \
    -I/home/fitze/simtk/opensim320/sdk/include \
    -I/home/fitze/simtk/opensim320/sdk/include/SimTK/include \
    -L/home/fitze/simtk/opensim320/lib \
    -losimSimulation \
    -losimActuators \
    -losimCommon \
    -losimTools \
    -losimAnalyses \
    -losimLepton \
    -lSimTKmath \
    -lSimTKcommon \
    -lSimTKsimbody

