#include "capture-malla.hpp"

bool flagExit = false;
bool flagRunOption = false;
Option option = None;


void registerCallbacks(open3d::visualization::VisualizerWithKeyCallback & vis) {
    using namespace open3d;

    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
                            [&](visualization::Visualizer *vis) {
                                flagExit = true;
                                return true;
                            });

    vis.RegisterKeyCallback(GLFW_KEY_Q,
                            [&](visualization::Visualizer *vis) {
                                flagRunOption = true;
                                option = Scan10sFast;
                                return true;
                            });

    vis.RegisterKeyCallback(GLFW_KEY_W,
                        [&](visualization::Visualizer *vis) {
                            flagRunOption = true;
                            option = Scan10sDetail;
                            return true;
                        });

    vis.RegisterKeyCallback(GLFW_KEY_E,
                            [&](visualization::Visualizer *vis) {
                                flagRunOption = true;
                                option = Scan30sFast;
                                return true;
                            });

    vis.RegisterKeyCallback(GLFW_KEY_R,
                            [&](visualization::Visualizer *vis) {
                                flagRunOption = true;
                                option = Scan30sDetail;
                                return true;
                            });

    vis.RegisterKeyCallback(GLFW_KEY_ENTER,
                        [&](visualization::Visualizer *vis) {
                            flagRunOption = true;
                            option = InstantCapture;
                            return true;
                        });

};


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



std::shared_ptr<open3d::geometry::PointCloud> createPointCloudFromRGBD(
    open3d::geometry::RGBDImage & imgRGBD,
    open3d::t::io::RGBDVideoMetadata & metadata
)
{
    using namespace open3d;

    std::cout << "---> Creando nube de puntos a partir de imagen RGBD" << std::endl;
    // Características de la instantánea:
    std::cout << "Píxeles de color: " << imgRGBD.color_.width_ <<"x"<<imgRGBD.color_.height_ << std::endl;
    std::cout << "Píxeles de profundidad: " << imgRGBD.depth_.width_ <<"x"<<imgRGBD.depth_.height_ << std::endl;

    // Transformación de la imagen RGBD para que la conversión a nube de puntos funcione
    // (la imagen de profundidad ha de tener 4 bytes por píxel, por algún motivo que desconozco)
    auto imgPointCloud = open3d::geometry::RGBDImage::CreateFromColorAndDepth(imgRGBD.color_, imgRGBD.depth_);
    (*imgPointCloud).color_ = imgRGBD.color_;
    std::cout << "Número de canales de profundidad: " << (*imgPointCloud).depth_.num_of_channels_ << std::endl;
    std::cout << "Bytes por canal de profundidad: " << (*imgPointCloud).depth_.bytes_per_channel_ << std::endl;

    // Corregimos la orientación del objeto generado
    Eigen::Matrix4d correctOrientation;
    correctOrientation << 1, 0, 0, 0,
                          0,  -1, 0, 0,
                          0,  0, -1, 0,
                          0,  0, 0, 1;

    return geometry::PointCloud::CreateFromRGBDImage(*imgPointCloud, metadata.intrinsics_, correctOrientation);
}


std::shared_ptr<open3d::geometry::TriangleMesh> createMeshFromPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> & pointCloud
)
{
    using namespace open3d;

    // Lanzamiento del algoritmo Poisson (parámetro depth por defecto = 8; subirlo puede aumentar la definición)
    std::cout << "---> Creando malla a partir de una nube de puntos mediante el algoritmo Poisson" << std::endl;
    auto poissonResult = geometry::TriangleMesh::CreateFromPointCloudPoisson(*(pointCloud));
    auto malla_ptr = std::get<0>(poissonResult);
    std::vector<double> densities = std::get<1>(poissonResult);

    // Limpieza de los vértices con baja densidad (han sido creados por pocos puntos de la nube original)
    // Calculamos el percentil 5 (0.05) de las densidades (será maxDensity), y todos los vértices que queden
    // por debajo serán eliminados
    std::cout << "Limpiando vértices de poca densidad" << std::endl;
    std::vector<double> densitiesCopy(densities);
    double percentile = 0.05;
    std::size_t index = static_cast<std::size_t>((densitiesCopy.size() - 1) * percentile);
    std::nth_element(densitiesCopy.begin(), densitiesCopy.begin() + index, densitiesCopy.end());
    auto maxDensity = densitiesCopy[index];
    std::vector<bool> verticesToRemove(densities.size());
    for (int i = 0; i < densities.size(); i++) {
        verticesToRemove[i] = densities[i] < maxDensity;
    }
    (*malla_ptr).RemoveVerticesByMask(verticesToRemove);

    return malla_ptr;

}



void initRealsense(open3d::t::io::RealSenseSensor & sensor) {
    using namespace open3d;

    // Apertura de la RealSense
    static const std::unordered_map<std::string, std::string> configValues =
    {
        {"serial", ""}, // Se escoge el primer dispositivo disponible
        {"color_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de color
        {"color_resolution", "640,480"}, // RealSense escogerá la resolución (la máxima)
        {"depth_format", "RS2_FORMAT_ANY"}, // Formato de pixel para fotogramas de profundidad
        {"depth_resolution", "640,480"}, // RealSense escogerá la resolución (la máxima)
        {"fps", "10"}, // Tasa de fotogramas para color y profundidad
        {"visual_preset", "RS2_SR300_VISUAL_PRESET_OBJECT_SCANNING"} // Preset por optimizado para el escaneo
    };
    sensor.InitSensor(t::io::RealSenseSensorConfig(configValues), 0);

}



void preprocessPointCloud(open3d::geometry::PointCloud & pc, double voxelSize, bool computeNormals) {
    using namespace open3d;

    pc = *pc.VoxelDownSample(voxelSize);
    //pc.EstimateNormals();
    //pc.OrientNormalsTowardsCameraLocation();
    if (computeNormals)
        pc.EstimateNormals(geometry::KDTreeSearchParamHybrid(voxelSize * 2, 30));

}



Eigen::Matrix4d_u simpleICPRegistration(open3d::geometry::PointCloud & source, open3d::geometry::PointCloud & target) {
    using namespace open3d;

    auto registrationResult = pipelines::registration::RegistrationICP(
            source,
            target,
            0.1,
            Eigen::Matrix4d::Identity(),
            // variante del algoritmo más veloz que por defecto point to point:
            pipelines::registration::TransformationEstimationPointToPlane()
        );
     return registrationResult.transformation_;
}


Eigen::Matrix4d_u optimusICPRegistration(open3d::geometry::PointCloud source, open3d::geometry::PointCloud target, bool fast) {
    using namespace open3d;


    const  static double voxelRadius [] = {0.04, 0.02, 0.01};
    const static int maxIter [] = {50, 30, 14};
    Eigen::Matrix4d_u currentTransformation = Eigen::Matrix4d::Identity();
    int times;
    if (fast)
        times = 2;
    else
        times = 3;

    for (int i = 0; i<times; i++) {
        auto vr = voxelRadius[i];
        auto mi = maxIter[i];
        auto sourceDown = source;
        auto targetDown = target;
        preprocessPointCloud(sourceDown, vr);
        preprocessPointCloud(targetDown, vr);
        auto registrationResult = pipelines::registration::RegistrationColoredICP(
            sourceDown,
            targetDown,
            vr,
            currentTransformation,
            pipelines::registration::TransformationEstimationForColoredICP(),
            pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, mi)
        );
        currentTransformation = registrationResult.transformation_;
    };
    return currentTransformation;

}


open3d::geometry::PointCloud mergePointCloudsICP(std::vector<open3d::geometry::PointCloud> & pointClouds, bool fast) {
    // Aplicación del algoritmo de registración a todas las nubes teniendo la primera como base

    double voxelSize;
    if (fast)
        voxelSize = 0.02;
    else
        voxelSize = 0.01;

    auto half1 = pointClouds[0];
    for (int i = 1; i < pointClouds.size() /2; i++)
    {
        std::cout << "*Empezando registración " << i <<std::endl;
        Eigen::Matrix4d_u transformation;
        try{
            transformation = optimusICPRegistration(half1, pointClouds[i], fast);
        } catch (const std::exception& ex) {
            std::cerr << ex.what() << std::endl
                << "No se ha podido registrar una nube. Omitiendo." << std::endl;
            continue;
        }
        half1.Transform(transformation);
        half1 += pointClouds[i];
        preprocessPointCloud(half1, voxelSize);
    }

    auto half2 = pointClouds[pointClouds.size() -1];
    for (int i = pointClouds.size() - 2; i >= pointClouds.size() / 2; i--)
    {
        std::cout << "*Empezando registración " << i <<std::endl;
        Eigen::Matrix4d_u transformation;
        try{
            transformation = optimusICPRegistration(half2, pointClouds[i], fast);
        } catch (const std::exception& ex) {
            std::cerr << ex.what() << std::endl
                << "No se ha podido registrar una nube. Omitiendo." << std::endl;
            continue;
        }
        half2.Transform(transformation);
        half2 += pointClouds[i];
        preprocessPointCloud(half2, voxelSize);
    }

    std::cout << "Fusión de las mitades " <<std::endl;
    Eigen::Matrix4d_u transformation;
    try{
        transformation = optimusICPRegistration(half1, half2, fast);
        half1.Transform(transformation);
        half1 += half2;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl
            << "No se ha podido registrar una nube. ERROR." << std::endl;
    }

    preprocessPointCloud(half1, voxelSize);

    return half1;
}




void instantCapture(open3d::t::io::RealSenseSensor & rs,
                    open3d::visualization::VisualizerWithKeyCallback & vis,
                    open3d::t::io::RGBDVideoMetadata & metadata)
{
    using namespace open3d;


    // Visualización en tiempo real de la cámara
    auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
    auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
            &imgRGBD, [](geometry::RGBDImage *) {});

    auto pointCloudPtr = createPointCloudFromRGBD(imgRGBD, metadata);
    bool writePointCloudSuccess = io::WritePointCloud("../archivos/nube_de_puntos.ply", *pointCloudPtr);
    if (writePointCloudSuccess) {
        std::cout << "Nube de puntos guardada" << std::endl;
    } else {
        std::cerr << "Error guardando la nube de puntos" << std::endl;
    }
    preprocessPointCloud(*pointCloudPtr, 0.01);
    (*pointCloudPtr).EstimateNormals();
    // (*pointCloud).OrientNormalsConsistentTangentPlane(4); // La mejor orientación de normales, peor es MUY costosa
    (*pointCloudPtr).OrientNormalsTowardsCameraLocation();

    auto meshPtr = createMeshFromPointCloud(pointCloudPtr);

    if (io::WriteTriangleMesh("../archivos/malla.ply", *meshPtr)) {
        std::cout << "Malla bien" <<std::endl;
    } else {
        std::cerr << "Malla mal" << std::endl;
    };

    visualization::DrawGeometries({meshPtr}, "Malla");
    visualization::DrawGeometries({pointCloudPtr}, "Nube de puntos");

}




void scanScene(open3d::t::io::RealSenseSensor & rs,
               open3d::visualization::VisualizerWithKeyCallback & vis,
               open3d::t::io::RGBDVideoMetadata & metadata,
               int frames,
               bool fast)
{
    using namespace open3d;


    vis.ClearGeometries();

    // Visualización en tiempo real de la cámara
    auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
    auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
            &imgRGBD, [](geometry::RGBDImage *) {});
    vis.AddGeometry(imRGBDPtr);

    // Temporización (4 nubes/s)
    const auto frame_interval = std::chrono::duration<double>(1. / 4.0);
    auto last_frame_time = std::chrono::steady_clock::now();
    int remainingPointClouds = frames;

    // Vector de nubes de puntos
    std::vector<geometry::PointCloud> pointClouds {};

    while(remainingPointClouds > 0) {

        // Generación de la nube de puntos
        auto pointCloud = *createPointCloudFromRGBD(imgRGBD, metadata);
        pointCloud.EstimateNormals();
        pointCloud.OrientNormalsTowardsCameraLocation();
        pointClouds.push_back(pointCloud);
        remainingPointClouds--;

        // Captura y visualización del frame
        vis.UpdateGeometry();
        vis.UpdateRender();
        std::this_thread::sleep_until(last_frame_time + frame_interval);
        last_frame_time = std::chrono::steady_clock::now();
        imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
        vis.PollEvents();
    }
    rs.StopCapture();

    // if (flagExit) return; // Cierre del programa si se ha pulsado ESC

    auto mergedPointCloud = std::make_shared<geometry::PointCloud>(
        mergePointCloudsICP(pointClouds, fast)
    );

    visualization::DrawGeometries({mergedPointCloud}, "Escaneo");
    auto mesh = createMeshFromPointCloud(mergedPointCloud);
    visualization::DrawGeometries({mesh}, "Malla escaneada");


}



int main(int argc, char *argv[]) {
    // realsenseRecord();
    // readBagFile();
    // visualizeMesh();
    // createGui();
    // requestFileToSave();
    // instantCapture();
    // scanScene(40, true); // 4 frames por segundo
    using namespace open3d;
    open3d::t::io::RealSenseSensor::ListDevices();

    t::io::RealSenseSensor rs;
    initRealsense(rs);
    auto metadata = rs.GetMetadata();
    rs.StartCapture();

    // CREACIÓN DE LA VENTANA
    visualization::VisualizerWithKeyCallback vis;
    visualization::SetGlobalColorMap(
            visualization::ColorMap::ColorMapOption::Gray);
    registerCallbacks(vis);
    vis.CreateVisualizerWindow("CaptureMalla");


    // Visualización en tiempo real de la cámara
    auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
    auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
            &imgRGBD, [](geometry::RGBDImage *) {});
    vis.AddGeometry(imRGBDPtr);

    // Temporización (10 fps)
    const auto frame_interval = std::chrono::duration<double>(1. / 10.0);
    auto last_frame_time = std::chrono::steady_clock::now();


    while(!flagExit && !flagRunOption > 0) {

        vis.UpdateGeometry();
        vis.UpdateRender();
        std::this_thread::sleep_until(last_frame_time + frame_interval);
        last_frame_time = std::chrono::steady_clock::now();
        imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
        vis.PollEvents();
    }


    if (flagExit) return 0;// Cierre del programa si se ha pulsado ESC

    switch (option) {
        case InstantCapture :
            std::cout << "Captura instantánea" << std::endl;
            instantCapture(rs, vis, metadata);
            break;

        case Scan10sFast :
            std::cout << "Scanner de 10 segundos, modo velocidad" << std::endl;
            scanScene(rs, vis, metadata, 40, true);
            break;

        case Scan10sDetail:
            std::cout << "Scanner de 10 segundos, modo detalle" << std::endl;
            scanScene(rs, vis, metadata, 40, false);
            break;

        case Scan30sFast:
            std::cout << "Scanner de 30 segundos, modo velocidad" << std::endl;
            scanScene(rs, vis, metadata, 120, true);
            break;

        case Scan30sDetail:
            std::cout << "Scanner de 30 segundos, modo detalle" << std::endl;
            scanScene(rs, vis, metadata, 120, false);
            break;
        default:
            std::cerr << "Error seleccionando la opción" << std::endl;
            break;
    }

    rs.StopCapture();

    return 0;
}
