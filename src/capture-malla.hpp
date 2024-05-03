#include "open3d/Open3D.h"
#include <iostream>
#include <tinyfiledialogs.h>
#include <cstring>


/**
 * Opciones del programa que puede seleccionar el usuario
 */
enum Option {
    None,
    InstantCapture,
    Scan10sFast,
    Scan10sDetail,
    Scan30sFast,
    Scan30sDetail
};


/**
 * Asocia una opción a cada botón
 */
void registerCallbacks(open3d::visualization::VisualizerWithKeyCallback & vis);

/*
 * Produce una grabación usando la cámara Intel RealSense. Grabará 2 segundos (60 fotogramas), y
 * el resultado se almacena un archivo ".bag", el cual contendrá datos de color y profundidad.
 * */
void realsenseRecord();

void readBagFile();

/*
 * Visualiza una malla 3D que se se encuentra en un archivo con un formato de malla 3D válido,
 * en este caso PLY.
 */
void visualizeMesh();

void createGui();

/**
 * Abre un diálogo para solicitar la ruta donde se guardará un archivo ".ply".
 * Retorna la ruta elegida, o cadena vacía si el usuario ha cancelado.
 */
std::string requestFileToSave();


/**
 * Retorna un shared_ptr a una nube de puntos que se crea a partir de la imagen RGBD,
 * teniendo en cuenta las características de la cámara.
 */
std::shared_ptr<open3d::geometry::PointCloud> createPointCloudFromRGBD(
    open3d::geometry::RGBDImage & imgRGBD,
    open3d::t::io::RGBDVideoMetadata & metadata
);

/*
 * Lanza el algoritmo Poisson para crear una malla 3D a partir de una nube de puntos
 * (es el que mejor resultado nos ha dado). Se asume que la nube de puntos tiene normales,
 * La consistencia de las normales afectará a la calidad de la malla resultante.
 */
std::shared_ptr<open3d::geometry::TriangleMesh> createMeshFromPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> & pointCloud
);


/* Inicializa la Realsense con la configuración más óptima para el escaneo
 */
void initRealsense(open3d::t::io::RealSenseSensor & sensor);


void preprocessPointCloud(open3d::geometry::PointCloud & pc, double voxelSize, bool computeNormals=false);

void instantCapture(open3d::t::io::RealSenseSensor & rs, open3d::visualization::VisualizerWithKeyCallback & vis);

Eigen::Matrix4d_u simpleICPRegistration(open3d::geometry::PointCloud & source, open3d::geometry::PointCloud & target);

Eigen::Matrix4d_u optimusICPRegistration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, bool fast);

open3d::geometry::PointCloud mergePointCloudsICP(std::vector<open3d::geometry::PointCloud> & pointClouds, bool fast);

void scanScene(int frames, bool fast);
