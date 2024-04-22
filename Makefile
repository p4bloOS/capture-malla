OPEN3D_LIBS=$(shell pkg-config --cflags --libs Open3D)

SRCDIR = src
INCDIR = include
BINDIR = bin

$(BINDIR)/ejemplo_Draw: $(SRCDIR)/ejemplo_Draw.cpp
	$(CXX) $(SRCDIR)/ejemplo_Draw.cpp -o $@ $(OPEN3D_LIBS)
