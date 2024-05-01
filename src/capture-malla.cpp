#include "open3d/Open3D.h"
#include <GLFW/glfw3.h>
#include <tinyfiledialogs.h>
#include <cstring>


/*
 * Produce una grabación usando la cámara Intel RealSense. Grabará 2 segundos (60 fotogramas), y
 * el resultado se almacena un archivo ".bag", el cual contendrá datos de color y profundidad.
 * */
void realsenseRecord() {
    using namespace open3d::t;
    std::unordered_map<std::string, std::string> configValues =
    {
        {"serial", ""}, // Se escoge el primer dispositivo disponible
        {"color_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de color
        {"color_resolution", "1920,1080"},
        {"depth_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de profundidad
        {"depth_resolution", "640,480"},
        {"fps", "30"}, // Tasa de fotogramas para color y profundidad
        {"visual_preset", ""} // Preset por defecto
    };
    const std::string bagFilename = "secuencia.bag";

    io::RealSenseSensor rs;
    rs.InitSensor(io::RealSenseSensorConfig(configValues), 0, bagFilename);
    rs.StartCapture(true);  // true: start recording with capture

    for(size_t fid = 0; fid<60; ++fid) {
        geometry::RGBDImage image = rs.CaptureFrame(true, true);
        // process im_rgbd.depth_ and im_rgbd.color_ ...
    }
    rs.StopCapture();
}

void readBagFile() {
    using namespace open3d;

    t::io::RSBagReader bag_reader;
    bag_reader.Open("../archivos/secuencia.bag");
    auto metadata = bag_reader.GetMetadata();
    /*
    Json::Value jsonMetadata;
    metadata.ConvertToJsonValue(jsonMetadata);
    std::cout << open3d::utility::JsonToString(jsonMetadata) << std::endl;
    */
    // VENTANA
    visualization::VisualizerWithKeyCallback vis;
    visualization::SetGlobalColorMap(
            visualization::ColorMap::ColorMapOption::Gray);
    bool flag_exit;
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flag_exit = true;
                                return true;
                            });
    vis.CreateVisualizerWindow("Open3D Intel RealSense bag player");


    const auto frame_interval = std::chrono::duration<double>(1. / metadata.fps_);
    auto last_frame_time = std::chrono::steady_clock::now();

    bool is_geometry_added = false;
    auto im_rgbd = bag_reader.NextFrame().ToLegacy();

    int frameNum = 0;
    while (!bag_reader.IsEOF() && !flag_exit) {
        frameNum ++;
        // Guardamos un fotograma para hacer pruebas:
        if (frameNum == 45) {
            const std::string colorFileName = "../archivos/color_image.png";
            const std::string depthFileName = "../archivos/depth_image.png";
            io::WriteImage(colorFileName, im_rgbd.color_);
            io::WriteImage(depthFileName, im_rgbd.depth_);
        }
        //im_rgbd.depth_;

        // process im_rgbd.depth_ and im_rgbd.color_

        // create shared_ptr with no-op deleter for stack RGBDImage
        auto ptr_im_rgbd = std::shared_ptr<geometry::RGBDImage>(
            &im_rgbd, [](geometry::RGBDImage *) {});

        // Improve depth visualization by scaling
        /* im_rgbd.depth_.LinearTransform(0.25); */
        if (ptr_im_rgbd->IsEmpty()) continue;

        if (!is_geometry_added) {
                vis.AddGeometry(ptr_im_rgbd);
                is_geometry_added = true;
            }


        vis.UpdateGeometry();
        vis.UpdateRender();
        std::this_thread::sleep_until(last_frame_time + frame_interval);
        last_frame_time = std::chrono::steady_clock::now();
        im_rgbd = bag_reader.NextFrame().ToLegacy();
        vis.PollEvents();

    }
    bag_reader.Close();
}



/*
 * Visualiza una malla 3D que se se encuentra en un archivo con un formato de malla 3D válido,
 * en este caso PLY.
 */
void visualizeMesh() {

    using namespace open3d;

    // Open3D gestiona la visualización de una geometría mediante un puntero compartido (shared_ptr)
    // que apunta a dicha geometría
    auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
    const std::string meshFilename = "../archivos/malla_de_prueba.ply";
    if (io::ReadTriangleMesh(meshFilename, *mesh_ptr)) {
        utility::LogInfo("Malla leída correctamente: {}", meshFilename);
    } else {
        utility::LogWarning("Fallo al leer la malla: {}", meshFilename);
        return;
    }
    mesh_ptr->ComputeVertexNormals();

    // Se lanza la visualización en una ventana:
    visualization::DrawGeometries({mesh_ptr}, "Malla");

}


void createGui( ) {
    using namespace open3d;


    // secuencia bag
    t::io::RSBagReader bag_reader;
    bag_reader.Open("../archivos/20240408_200556.bag");
    auto metadata = bag_reader.GetMetadata();

    // Malla
    auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
    const std::string meshFilename = "assets/instrucciones.ply";
    if (io::ReadTriangleMesh(meshFilename, *mesh_ptr)) {
        utility::LogInfo("Malla leída correctamente: {}", meshFilename);
    } else {
        utility::LogWarning("Fallo al leer la malla: {}", meshFilename);
        return;
    }
    mesh_ptr->ComputeVertexNormals();

    // VENTANA
    visualization::VisualizerWithKeyCallback vis;
    visualization::SetGlobalColorMap(
            visualization::ColorMap::ColorMapOption::Gray);
    bool flag_exit;
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flag_exit = true;
                                return true;
                            });
    vis.CreateVisualizerWindow("INTENTO DE INTERFAZ");

    // Introducción de objetos en la interfaz
    vis.AddGeometry(mesh_ptr);


    const auto frame_interval = std::chrono::duration<double>(1. / metadata.fps_);
    auto last_frame_time = std::chrono::steady_clock::now();

    bool is_geometry_added = false;
    auto im_rgbd = bag_reader.NextFrame().ToLegacy();
    while (!bag_reader.IsEOF() && !flag_exit) {

        // Así evitamos que se pueda mover la malla que muestra las instrucciones
        vis.GetViewControl().Reset();
        // process im_rgbd.depth_ and im_rgbd.color_

        // create shared_ptr with no-op deleter for stack RGBDImage
        auto ptr_im_rgbd = std::shared_ptr<geometry::RGBDImage>(
            &im_rgbd, [](geometry::RGBDImage *) {});


        // Improve depth visualization by scaling
        // im_rgbd.depth_.LinearTransform(0.25);
        if (ptr_im_rgbd->IsEmpty()) continue;

        if (!is_geometry_added) {
                vis.AddGeometry(ptr_im_rgbd);
                is_geometry_added = true;
            }


        vis.UpdateGeometry();
        vis.UpdateRender();
        std::this_thread::sleep_until(last_frame_time + frame_interval);
        last_frame_time = std::chrono::steady_clock::now();
        im_rgbd = bag_reader.NextFrame().ToLegacy();
        vis.PollEvents();

    }
    bag_reader.Close();

}


/**
 * Abre un diálogo para solicitar la ruta donde se guardará un archivo ".ply".
 * Retorna la ruta elegida, o cadena vacía si el usuario ha cancelado.
 */
std::string requestFileToSave() {

    const char * filterPatterns[2] = {"*.ply", "*.PLY"};
    char const *filePath = tinyfd_saveFileDialog("Elige el archivo que guardar", "", 2,
        filterPatterns, NULL);
    if (filePath == nullptr) {
        std::cout << "Guardado de archivo cancelado." << std::endl;
        return "";
    }


    std::string filePathCorrected;
    size_t path_len = strlen(filePath);
    if ( path_len < 4)
    {
        filePathCorrected = std::string(filePath) + ".ply";
    }
    else {
        const char *suffix = filePath + path_len - 4;
        if (strcmp(suffix, ".ply") == 0 || strcmp(suffix, ".PLY") == 0) {
            filePathCorrected = filePath;
        } else {
            filePathCorrected = filePath + std::string(".ply");
        };
    };
    std::cout << "Ruta para guardar el archivo: " << filePathCorrected << std::endl;
    return filePathCorrected;

}


void instantCapture() {
    using namespace open3d;

    // Apertura de la RealSense
    std::unordered_map<std::string, std::string> configValues =
    {
        {"serial", ""}, // Se escoge el primer dispositivo disponible
        {"color_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de color
        {"color_resolution", "0,0"}, // RealSense escogerá la resolución (la máxima)
        {"depth_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de profundidad
        {"depth_resolution", "0,0"}, // RealSense escogerá la resolución (la máxima)
        {"fps", "30"}, // Tasa de fotogramas para color y profundidad
        {"visual_preset", "RS2_SR300_VISUAL_PRESET_OBJECT_SCANNING"} // Preset por optimizado para el escaneo
    };
    t::io::RealSenseSensor rs;
    rs.InitSensor(t::io::RealSenseSensorConfig(configValues), 0);
    auto metadata = rs.GetMetadata();
    rs.StartCapture();


    // CREACIÓN DE LA VENTANA
    visualization::VisualizerWithKeyCallback vis;
    visualization::SetGlobalColorMap(
            visualization::ColorMap::ColorMapOption::Gray);
    bool flag_exit = false;
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flag_exit = true;
                                return true;
                            });
    vis.CreateVisualizerWindow("Instant Capture");
    bool flag_capture;
    vis.RegisterKeyCallback(GLFW_KEY_ENTER,
                            [&](visualization::Visualizer *vis) {
                                flag_capture = true;
                                return true;
                            });


    // Visualización en tiempo real de la cámara
    auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
    auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
            &imgRGBD, [](geometry::RGBDImage *) {});
    vis.AddGeometry(imRGBDPtr);
    while(!flag_capture && !flag_exit) {
        imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
        vis.UpdateGeometry();
        vis.UpdateRender();
        vis.PollEvents();
    }
    rs.StopCapture();

    if (flag_exit) return; // Cierre del programa si se ha pulsado ESC

    // Características de la instantánea:
    std::cout << "Píxeles de color: " << imgRGBD.color_.width_ <<"x"<<imgRGBD.color_.height_ << std::endl;
    std::cout << "Píxeles de profundidad: " << imgRGBD.depth_.width_ <<"x"<<imgRGBD.depth_.height_ << std::endl;

    // Transformación de la imagen RGBD para que la conversión a nube de puntos funcione
    // (la imagen de profundidad ha de tener 4 bytes por píxel, por algún motivo que desconozco)
    auto imgPointCloud = open3d::geometry::RGBDImage::CreateFromColorAndDepth(imgRGBD.color_, imgRGBD.depth_);
    (*imgPointCloud).color_ = imgRGBD.color_;
    std::cout << "Número de canales de profundidad: " << (*imgPointCloud).depth_.num_of_channels_ << std::endl;
    std::cout << "Bytes por canal de profundidad: " << (*imgPointCloud).depth_.bytes_per_channel_ << std::endl;


    // Creación de la nube de puntos
    auto pointCloud = geometry::PointCloud::CreateFromRGBDImage(*imgPointCloud, metadata.intrinsics_);
    visualization::DrawGeometries({pointCloud}, "Nube de puntos");


}






int main(int argc, char *argv[]) {
    open3d::t::io::RealSenseSensor::ListDevices();
    // realsenseRecord();
    // readBagFile();
    // visualizeMesh();
    // createGui();
    // requestFileToSave();
    instantCapture();
    return 0;
}
