OPEN3D_LIBS=$(shell pkg-config --cflags --libs Open3D)
JSON_LIB=-ljsoncpp

SRCDIR = src
INCDIR = include
BINDIR = bin
EXAMPLES_DIR = src/examples


$(BINDIR)/capture-malla: $(SRCDIR)/capture-malla.cpp $(BINDIR)/tinyfiledialogs.o
	$(CXX) $(SRCDIR)/capture-malla.cpp $(BINDIR)/tinyfiledialogs.o -o $@ $(OPEN3D_LIBS) $(JSON_LIB) -I./$(INCDIR)


$(BINDIR)/tinyfiledialogs.o: $(INCDIR)/tinyfiledialogs.c $(INCDIR)/tinyfiledialogs.h | $(BINDIR)
	$(CXX) -c $(INCDIR)/tinyfiledialogs.c -o $(BINDIR)/tinyfiledialogs.o

$(BINDIR):
	mkdir -p $(BINDIR)




FileDialog: $(EXAMPLES_DIR)/FileDialog.cpp $(BINDIR)/tinyfiledialogs.o $(INCDIR)/tinyfiledialogs.h
	$(CXX) $(EXAMPLES_DIR)/FileDialog.cpp $(BINDIR)/tinyfiledialogs.o -o $(BINDIR)/FileDialog $(OPEN3D_LIBS) -I./$(INCDIR)


Draw: $(EXAMPLES_DIR)/Draw.cpp
	$(CXX) $(EXAMPLES_DIR)/Draw.cpp -o $(BINDIR)/Draw $(OPEN3D_LIBS)

RealSenseBagReader: $(EXAMPLES_DIR)/RealSenseBagReader.cpp
	$(CXX) $(EXAMPLES_DIR)/RealSenseBagReader.cpp -o $(BINDIR)/RealSenseBagReader $(OPEN3D_LIBS) $(JSON_LIB)

Visualizer: $(EXAMPLES_DIR)/Visualizer.cpp
	$(CXX) $(EXAMPLES_DIR)/Visualizer.cpp -o $(BINDIR)/Visualizer $(OPEN3D_LIBS)

MultipleWindows: $(EXAMPLES_DIR)/MultipleWindows.cpp
	$(CXX) $(EXAMPLES_DIR)/MultipleWindows.cpp  -o $(BINDIR)/MultipleWindows $(OPEN3D_LIBS)

TOdometryRGBD: $(EXAMPLES_DIR)/TOdometryRGBD.cpp
	$(CXX) $(EXAMPLES_DIR)/TOdometryRGBD.cpp -o $(BINDIR)/TOdometryRGBD $(OPEN3D_LIBS)



