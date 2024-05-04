#include "capture-malla.hpp"
#include <open3d/geometry/Geometry.h>
#include <thread>

bool flagExit = false;
bool flagRunOption = false;
Option option = None;
std::shared_ptr<open3d::geometry::TriangleMesh> helpPanel;
bool panelIsVisible;



int main(int argc, char *argv[]) {
    using namespace open3d;

    // Inicialización del panel de ayuda (es una malla ubicada en assets)
    helpPanel = std::make_shared<geometry::TriangleMesh>();
    const std::string meshFilename = "assets/panel_ayuda.ply";
    if (io::ReadTriangleMesh(meshFilename, *helpPanel)) {
        std::cout << "Malla del panel de ayuda correctamente" << std::endl;
    } else {
        std::cerr << "Fallo al leer la malla del panel de ayuda: " << meshFilename << std::endl;
        return -1;
    }
    helpPanel->ComputeVertexNormals();

    open3d::t::io::RealSenseSensor::ListDevices();
    t::io::RealSenseSensor rs;
    initRealsense(rs);
    auto metadata = rs.GetMetadata();

    // CREACIÓN DE LA VENTANA
    visualization::VisualizerWithKeyCallback vis;
    visualization::SetGlobalColorMap(
            visualization::ColorMap::ColorMapOption::Gray);
    registerCallbacks(vis);
    vis.CreateVisualizerWindow("CaptureMalla");
    vis.AddGeometry(helpPanel);
    panelIsVisible = true;

    while(true) {

        rs.StartCapture(); // Apertura de la cámara

        // Visualización en tiempo real de la cámara
        auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
        auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
                &imgRGBD, [](geometry::RGBDImage *) {});
        vis.AddGeometry(imRGBDPtr);

        // Temporización (10 fps)const auto frame_interval = std::chrono::duration<double>(1. / 10.0);
        const auto frame_interval = std::chrono::duration<double>(1. / 10.0);
        auto last_frame_time = std::chrono::steady_clock::now();

        // Visualizamos la imagen de la cámara mientras no se seleccione ninguna opción
        while(!flagExit && !flagRunOption > 0) {

            vis.UpdateGeometry();
            vis.UpdateRender();
            std::this_thread::sleep_until(last_frame_time + frame_interval);
            last_frame_time = std::chrono::steady_clock::now();
            imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
            vis.PollEvents();
        }

        if (flagExit) {
            rs.StopCapture();
            return 0;
        }// Cierre del programa si se ha pulsado ESC

        flagRunOption = false;

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

            case Help:
                changeHelpVision(vis, imRGBDPtr);
                break;

            default:
                std::cerr << "Error seleccionando la opción" << std::endl;
                break;
        }

    } // El programa vuelve a atender opciones infinitamente

    return 0;
}



void instantCapture(open3d::t::io::RealSenseSensor & rs,
                    open3d::visualization::VisualizerWithKeyCallback & vis,
                    open3d::t::io::RGBDVideoMetadata & metadata)
{
    using namespace open3d;

    // Captura imagen
    auto imgRGBD = rs.CaptureFrame(true, true).ToLegacy();
    rs.StopCapture();
    auto imRGBDPtr = std::shared_ptr<geometry::RGBDImage>(
        &imgRGBD, [](geometry::RGBDImage *) {});

    // Características de la instantánea:
    std::cout << "Píxeles de color: " << imgRGBD.color_.width_ <<"x"<<imgRGBD.color_.height_ << std::endl;
    std::cout << "Píxeles de profundidad: " << imgRGBD.depth_.width_ <<"x"<<imgRGBD.depth_.height_ << std::endl;

    // Generación de la nube de puntos
    std::cout << "Creando nube de puntos" << std::endl;
    auto pointCloudPtr = createPointCloudFromRGBD(imgRGBD, metadata);
    preprocessPointCloud(*pointCloudPtr, 0.01);
    pointCloudPtr->EstimateNormals();
    pointCloudPtr->OrientNormalsTowardsCameraLocation();
    // (*pointCloud).OrientNormalsConsistentTangentPlane(4); // La mejor orientación de normales, peor es MUY costosa

    submit3DObject(pointCloudPtr, vis);
}



void scanScene(open3d::t::io::RealSenseSensor & rs,
               open3d::visualization::VisualizerWithKeyCallback & vis,
               open3d::t::io::RGBDVideoMetadata & metadata,
               int frames,
               bool fast)
{
    using namespace open3d;


    if (panelIsVisible) {
        changeHelpVision(vis, std::make_shared<geometry::TriangleMesh>(geometry::TriangleMesh()));
    }
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

    vis.ClearGeometries(); vis.UpdateGeometry(); vis.UpdateRender(); vis.PollEvents();

    auto mergedPointCloud = std::make_shared<geometry::PointCloud>(
        mergePointCloudsICP(pointClouds, fast)
    );

    submit3DObject(mergedPointCloud, vis);
}



void submit3DObject(std::shared_ptr<open3d::geometry::PointCloud> pointCloudPtr,
                    open3d::visualization::VisualizerWithKeyCallback & vis)
{
    using namespace open3d;

    if (panelIsVisible) {
        changeHelpVision(vis, std::make_shared<geometry::TriangleMesh>(geometry::TriangleMesh()));
    }
    vis.ClearGeometries();
    vis.AddGeometry(pointCloudPtr);
    const auto refreshInterval = std::chrono::duration<double>(1. / 30.0);
    auto last_frame_time = std::chrono::steady_clock::now();
    std::string fileName;
    bool continueTransformation = false;

    while(!flagExit && !continueTransformation) {

        if (flagRunOption) {
            flagRunOption = false;
            switch (option) {
                case SaveFile:
                    fileName = requestFileToSave();
                    if (fileName != "") {
                        if (io::WritePointCloud(fileName, *pointCloudPtr))
                        {
                            std::cout << "Nube de puntos guardada" << std::endl;
                        } else {
                            std::cerr << "Error guardando la nube de puntos" << std::endl;
                        }
                    };
                    break;
                case Help:
                    changeHelpVision(vis, pointCloudPtr);
                    break;
                case TransformToMesh:
                    continueTransformation = true;
                    break;
                default:
                    std::cerr << "Pulsada opción no permitida aquí" << std::endl;
                    break;
            }
        }

        vis.UpdateGeometry();
        vis.UpdateRender();
        vis.PollEvents();
        std::this_thread::sleep_until(last_frame_time + refreshInterval);
        last_frame_time = std::chrono::steady_clock::now();
    }

    if (flagExit) { // Abortar si se ha pulsado ESCAPE
        vis.ClearGeometries();
        flagExit = false;
        return;
    }

    if (panelIsVisible) {
        changeHelpVision(vis, std::make_shared<geometry::TriangleMesh>(geometry::TriangleMesh()));
    }
    vis.ClearGeometries();
    vis.UpdateGeometry();
    vis.UpdateRender();
    vis.PollEvents();

    // Transformación de la nube de puntos en una malla
    auto meshPtr = createMeshFromPointCloud(pointCloudPtr);

    vis.AddGeometry(meshPtr);
    while(!flagExit) {

        if (flagRunOption) {
            flagRunOption = false;
            switch (option) {
                case SaveFile:
                    fileName = requestFileToSave();
                    if (fileName != "") {
                        if (io::WriteTriangleMesh(fileName, *meshPtr))
                        {
                            std::cout << "Malla 3D guardada" << std::endl;
                        } else {
                            std::cerr << "Error guardando malla 3D" << std::endl;
                        }
                    };
                    break;
                case Help:
                    changeHelpVision(vis, meshPtr);
                    break;
                default:
                    std::cerr << "Pulsada opción no permitida aquí" << std::endl;
                    break;
            }
        }
        vis.UpdateGeometry();
        vis.UpdateRender();
        vis.PollEvents();
        std::this_thread::sleep_until(last_frame_time + refreshInterval);
        last_frame_time = std::chrono::steady_clock::now();
    }
    vis.ClearGeometries();
    if (panelIsVisible) {
        panelIsVisible = false;
        changeHelpVision(vis, std::make_shared<geometry::TriangleMesh>(geometry::TriangleMesh()));
    }
    flagExit = false;

}



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

    vis.RegisterKeyCallback(GLFW_KEY_H,
                    [&](visualization::Visualizer *vis) {
                        flagRunOption = true;
                        option = Help;
                        return true;
                    });

    vis.RegisterKeyCallback(GLFW_KEY_G,
                [&](visualization::Visualizer *vis) {
                    flagRunOption = true;
                    option = SaveFile;
                    return true;
                });

    vis.RegisterKeyCallback(GLFW_KEY_T,
                [&](visualization::Visualizer *vis) {
                    flagRunOption = true;
                    option = TransformToMesh;
                    return true;
                });
};



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


    // Transformación de la imagen RGBD para que la conversión a nube de puntos funcione
    // (la imagen de profundidad ha de tener 4 bytes por píxel, por algún motivo que desconozco)
    auto imgPointCloud = open3d::geometry::RGBDImage::CreateFromColorAndDepth(imgRGBD.color_,
                                                                              imgRGBD.depth_,
                                                                              1000,
                                                                              3,
                                                                              false);

    // (*imgPointCloud).color_ = imgRGBD.color_;
    // std::cout << "Número de canales de profundidad: " << (*imgPointCloud).depth_.num_of_channels_ << std::endl;
    // std::cout << "Bytes por canal de profundidad: " << (*imgPointCloud).depth_.bytes_per_channel_ << std::endl;

    // Corregimos la orientación del objeto generado (salía del revés)
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
    std::cout << "Creando malla a partir de una nube de puntos mediante el algoritmo Poisson" << std::endl;
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



void preprocessPointCloud(open3d::geometry::PointCloud & pc,
                          double voxelSize,
                          bool computeNormals)
{
    using namespace open3d;

    pc = *pc.VoxelDownSample(voxelSize);
    if (computeNormals)
        pc.EstimateNormals(geometry::KDTreeSearchParamHybrid(voxelSize * 2, 30));
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

    double voxelSize;
    if (fast)
        voxelSize = 0.02;
    else
        voxelSize = 0.01;

    std::cout << "Calculando normales de todas las nubes" << std::endl;
    for (auto & pointCloud  : pointClouds) {
        pointCloud.EstimateNormals();
        pointCloud.OrientNormalsTowardsCameraLocation();
        std::cout << "-" << std::flush;
    }
    std::cout << std::endl;

    std::cout << "Fusión" << std::endl;
    auto half1 = pointClouds[0];
    for (int i = 1; i < pointClouds.size() /2; i++)
    {
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
        std::cout << "*" << std::flush;
    }

    auto half2 = pointClouds[pointClouds.size() -1];
    for (int i = pointClouds.size() - 2; i >= pointClouds.size() / 2; i--)
    {
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
        std::cout << "*" << std::flush;
    }

    // Fusión de las mitades
    Eigen::Matrix4d_u transformation;
    try{
        transformation = optimusICPRegistration(half1, half2, fast);
        half1.Transform(transformation);
        half1 += half2;
        std::cout << "*" << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl
            << "No se ha podido registrar una nube. ERROR." << std::endl;
    }

    preprocessPointCloud(half1, voxelSize);

    return half1;
}



void changeHelpVision(open3d::visualization::VisualizerWithKeyCallback & vis,
                      std::shared_ptr<open3d::geometry::Geometry> geometryInView)
{
    if (!panelIsVisible)
    {
        std::cout << "Mostrando panel de ayuda" << std::endl;
        vis.ClearGeometries();
        vis.AddGeometry(helpPanel);
        vis.RemoveGeometry(helpPanel);
        vis.AddGeometry(helpPanel);
    } else
    {
        std::cout << "Ocultando panel de ayuda" << std::endl;
        vis.RemoveGeometry(helpPanel);
        vis.AddGeometry(geometryInView);
    }
    panelIsVisible = !panelIsVisible;
    return;
}


