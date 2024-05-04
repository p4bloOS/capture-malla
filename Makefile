OPEN3D_LIBS=$(shell pkg-config --cflags --libs Open3D)
JSON_LIB=-ljsoncpp

SRCDIR = src
INCDIR = include
BINDIR = bin
EXAMPLES_DIR = src/examples



################ COMPILACIÓN DEL PROGRAMA CAPTURE-MALLA: ################
#
# * Sitúate en el directorio raíz de este proyecto y ejecuta make para
#   obtener el binario.
#
# * El ejecutable resultanto se econtrará en la ruta bin/capture-malla
#
# * Necesita que la biblioteca Open3d esté instalada en el sistema y
#   su información sea recuperable por la herramienta pkg-config.
#
# * Depende de la biblioteca tinyfiledialogs, cuyo código fuente hemos
#   hemos incluido en el directorio include/ de este proyecto.
#

$(BINDIR)/capture-malla: $(SRCDIR)/capture-malla.cpp $(BINDIR)/tinyfiledialogs.o
	$(CXX) $(SRCDIR)/capture-malla.cpp $(BINDIR)/tinyfiledialogs.o -o $@ $(OPEN3D_LIBS) -I./$(INCDIR)


$(BINDIR)/tinyfiledialogs.o: $(INCDIR)/tinyfiledialogs.c $(INCDIR)/tinyfiledialogs.h | $(BINDIR)
	$(CXX) -c $(INCDIR)/tinyfiledialogs.c -o $(BINDIR)/tinyfiledialogs.o

$(BINDIR):
	mkdir -p $(BINDIR)
