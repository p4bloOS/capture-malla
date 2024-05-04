#include "open3d/Open3D.h"
#include <iostream>
#include <open3d/geometry/Geometry.h>
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
    Scan30sDetail,
    Help,
    SaveFile,
    TransformToMesh
};


/*
 * Simplifica la nube de puntos usando VoxelDownSample, y calcula sus normales si se indica.
 */
void preprocessPointCloud(open3d::geometry::PointCloud & pc, double voxelSize, bool computeNormals=false);


/*
 * Lleva a cabo una "Registración" (obtención de una matriz de transformación capaz de traducir la
 * nube "source" al sistema de coordenadas de la nube "target"). Invoca una versión mejorada de ICP
 * llamada "Colored ICP", que tiene en cuenta el color de los puntos. Se aplica el algoritmo 3 veces,
 * cada vez con mayor resolución, para mejorar el rendimiento.
 */
Eigen::Matrix4d_u optimusICPRegistration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, bool fast);


/**
 * Abre un diálogo para solicitar la ruta donde se guardará un archivo ".ply".
 * Retorna la ruta elegida, o cadena vacía si el usuario ha cancelado.
 */
std::string requestFileToSave();


/*
 * Lanza el algoritmo Poisson para crear una malla 3D a partir de una nube de puntos
 * (es el que mejor resultado nos ha dado). Se asume que la nube de puntos tiene normales,
 * La consistencia de las normales afectará a la calidad de la malla resultante.
 */
std::shared_ptr<open3d::geometry::TriangleMesh> createMeshFromPointCloud(std::shared_ptr<open3d::geometry::PointCloud> & pointCloud);


/*
 * Fusión de todas las nubes del vector en una sola. Se aplica el algoritmo de Registration a pares
 * de nubes, acumulando las transformaciones obtenidas en una nube resultado. Se realiza primero
 * una mitad del vector y luego otra para no acumular tantos puntos en la nube que se va
 * consiguiendo, y así ser más eficientes.
 */
open3d::geometry::PointCloud mergePointCloudsICP(std::vector<open3d::geometry::PointCloud> & pointClouds, bool fast);


/*
 * Retorna un shared_ptr a una nube de puntos que se crea a partir de la imagen RGBD,
 * teniendo en cuenta las características de la cámara.
 */
std::shared_ptr<open3d::geometry::PointCloud> createPointCloudFromRGBD(open3d::geometry::RGBDImage & imgRGBD, open3d::t::io::RGBDVideoMetadata & metadata);


/**
 * Asocia una opción a cada botón
 */
void registerCallbacks(open3d::visualization::VisualizerWithKeyCallback & vis);


/**
 * Muestra el panel de ayuda si está oculto. Lo oculta si está visible.
 */
void changeHelpVision(open3d::visualization::VisualizerWithKeyCallback & vis, std::shared_ptr<open3d::geometry::Geometry> geometryInView);

/*
 * Entrega al usuario el objeto 3D que ha generado. Se abre una visualización y se permite guardar
 * el objeto en un archivo. Primero se ofrece la nube de puntos, que el usuario puede transformar
 * en malla 3D si lo especifica
 */
void submit3DObject(std::shared_ptr<open3d::geometry::PointCloud> pointCloudPtr, open3d::visualization::VisualizerWithKeyCallback & vis);


/*
 * Realiza la opción "Captura instantánea", seleccionable por el usuario.
 */
void instantCapture(open3d::t::io::RealSenseSensor & rs,
                    open3d::visualization::VisualizerWithKeyCallback & vis,
                    open3d::t::io::RGBDVideoMetadata & metadata);


/*
 * Realiza cualquiera de las opciones de "Escaneo", seleccionables por el usuario
 */
void scanScene(open3d::t::io::RealSenseSensor & rs,
               open3d::visualization::VisualizerWithKeyCallback & vis,
               open3d::t::io::RGBDVideoMetadata & metadata,
               int frames,
               bool fast);


/*
 * Inicializa la Realsense con la configuración más óptima para el escaneo
 */
void initRealsense(open3d::t::io::RealSenseSensor & sensor);
