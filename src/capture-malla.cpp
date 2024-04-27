#include "open3d/Open3D.h"
#include <json/json.h>
#include <open3d/geometry/Image.h>
#include <open3d/io/ImageIO.h>
#include <open3d/utility/IJsonConvertible.h>
#include <open3d/visualization/gui/GLFWWindowSystem.h>
#include <open3d/visualization/gui/Window.h>

void realsenseRecord() {
    using namespace open3d::t;
    std::unordered_map<std::string, std::string> configValues =
    {
        {"serial", ""}, // Se escoge el primer dispositivo disponible
        {"color_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de color
        {"color_resolution", "1920,1080"}, // RealSense escogerá la resolución
        {"depth_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de profundidad
        {"depth_resolution", "640,480"}, // RealSense escogerá la resolución
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
    Json::Value jsonMetadata;
    metadata.ConvertToJsonValue(jsonMetadata);
    std::cout << open3d::utility::JsonToString(jsonMetadata) << std::endl;

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
    while (!bag_reader.IsEOF() && !flag_exit) {
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


void visualizeMesh() {

    using namespace open3d;
    auto mesh_ptr = std::make_shared<geometry::TriangleMesh>();
    const std::string meshFilename = "../archivos/foto_3d_pablo.ply";
    if (io::ReadTriangleMesh(meshFilename, *mesh_ptr)) {
        utility::LogInfo("Malla leída correctamente: {}", meshFilename);
    } else {
        utility::LogWarning("Fallo al leer la malla: {}", meshFilename);
        return;
    }
    mesh_ptr->ComputeVertexNormals();
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




int main(int argc, char *argv[]) {
    open3d::t::io::RealSenseSensor::ListDevices();
    // realsenseRecord();
    // readBagFile();
    // visualizeMesh();
    createGui();
    return 0;
}
