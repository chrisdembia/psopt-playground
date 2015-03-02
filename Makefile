include ../Makefile_linux.inc

MYTWOLINK = mytwolinkarm   $(SNOPT_WRAPPER)

MYTWOLINK_O = $(MYTWOLINK:%=$(EXAMPLESDIR)/%.o)

opensimtwolink.o:
	g++ -c -o opensimtwolink.o opensimtwolink.cpp \
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

mytwolink: $(MYTWOLINK_O) opensimtwolink.o $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS) \
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
	-lSimTKsimbody \
