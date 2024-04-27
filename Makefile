OPEN3D_LIBS=$(shell pkg-config --cflags --libs Open3D)
JSON_LIB=-ljsoncpp

SRCDIR = src
INCDIR = include
BINDIR = bin
EXAMPLES_DIR = src/examples

.PHONY: $(BINDIR)/ejemplo_Draw

$(BINDIR)/capture-malla: $(SRCDIR)/capture-malla.cpp | $(BINDIR)
	$(CXX) $(SRCDIR)/capture-malla.cpp -o $@ $(OPEN3D_LIBS) $(JSON_LIB)


Draw: $(EXAMPLES_DIR)/Draw.cpp | $(BINDIR)
	$(CXX) $(EXAMPLES_DIR)/Draw.cpp -o $(BINDIR)/Draw $(OPEN3D_LIBS)

RealSenseBagReader: $(EXAMPLES_DIR)/RealSenseBagReader.cpp | $(BINDIR)
	$(CXX) $(EXAMPLES_DIR)/RealSenseBagReader.cpp -o $(BINDIR)/RealSenseBagReader $(OPEN3D_LIBS) $(JSON_LIB)

Visualizer: $(EXAMPLES_DIR)/Visualizer.cpp | $(BINDIR)
	$(CXX) $(EXAMPLES_DIR)/Visualizer.cpp -o $(BINDIR)/Visualizer $(OPEN3D_LIBS)

MultipleWindows: $(EXAMPLES_DIR)/MultipleWindows.cpp | $(BINDIR)
	$(CXX) $(EXAMPLES_DIR)/MultipleWindows.cpp  -o $(BINDIR)/MultipleWindows $(OPEN3D_LIBS)

$(BINDIR):
	mkdir -p $(BINDIR)

