include ../Makefile_linux.inc

MYTWOLINK = mytwolinkarm   $(SNOPT_WRAPPER)

MYTWOLINK_O = $(MYTWOLINK:%=$(EXAMPLESDIR)/%.o)


mytwolink: $(MYTWOLINK_O) $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS)

