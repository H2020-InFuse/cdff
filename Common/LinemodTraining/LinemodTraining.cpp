/**
 * @addtogroup LinemodTraining
 * @{
 */

#include "linemod-wrapper/LinemodTrainingImpl.hpp"

int main(int argc, char *argv[])
{
    const cv::String keys = "{help               |            | ./LinemodTraining --CADmodel=<path to model without extension> [...]}"
                            "{CADmodel           |            | Path to the CAD model name, without extension.}"
                            "{isPLY              | false      | true for .ply, false for .obj}"
                            "{useDepth           | false      | true to use depth templates or not.}"
                            "{long               | -120:1:-60 | Longitude_min:Longitude_step:Longitude_max}"
                            "{lat                | 0:1:40     | Latitude_min:Latitude_step:Latitude_max}"
                            "{angle              | 0:1:0      | Angle_min:Angle_step:Angle_max}"
                            "{radius             | 1.5:0.1:3  | Radius_min:Radius_step:Radius_max}"
                            "{T_level0           | 5          | Parameter for sampling step T at level 0 of the pyramid in Linemod}"
                            "{T_level1           | 8          | Parameter for sampling step T at level 1 of the pyramid in Linemod}"
                            "{renderWindowWidth  | 640        | Width of the rendering window}"
                            "{renderWindowHeight | 480        | Height of the rendering window}"
                            "{fx                 | 682.559    | Camera focal length in x in pixel (for DLR stereo-cameras)}"
                            "{fy                 | 682.261    | Camera focal length in y in pixel (for DLR stereo-cameras)}"
                            "{cx                 | 349.218    | Camera principal point in x in pixel (for DLR stereo-cameras)}"
                            "{cy                 | 266.693    | Camera principal point in y in pixel (for DLR stereo-cameras)}"
                            "{saveTrainingImg    |            | Path to the directory where to save the rendered images, or empty}";

    cv::CommandLineParser parser(argc, argv, keys);

    parser.about("Generate training data for Linemod-based pose detection.\n"
                 "The program takes as input the path to a CAD model file (without the extension) and additional parameters specified by command line options.\n"
                 "All parameteres have a default value, please consult the help section for details on the parameters.\n"
                 "The program generates detection CAD model templates, that will allow an appropriate detector to identify the original CAD model object in RGB and depth images.\n"
                 "It implements the Linemod training as proposed in 'Model Based Training, Detection and Pose Estimation of Texture-Less 3D Objects in Heavily Cluttered Scenes' by Hinterstoisser et al.\n"
                 "The idea is to render multiple views of the object of interest and sampled on a sphere.\n"
                 "Color and depth modalities are extracted and saved along with the corresponding pose that generated the view.\n"
                 "For the detection part, a multimodal templates based approach is used to retrieve the closest trained view in the current image.");
    if (parser.has("help") || argc == 1)
    {
        parser.printMessage();
        return 0;
    }

    ///Command line parameters parsing///
    //CAD model parameters
    std::string CADmodel = parser.get<std::string>("CADmodel");
    bool isPLY = parser.get<bool>("isPLY");
    bool useDepth = parser.get<bool>("useDepth");

    std::cout << "Path to the CAD model file: " << CADmodel << std::endl;
    std::cout << "PLY format? " << isPLY << std::endl;
    std::cout << "Use depth modality? " << useDepth << std::endl;

    //Longitude, latitude, angle and radius parameters
    //Used to sample views on a sphere around the object of interest
    std::string long_param = parser.get<std::string>("long");
    std::string lat_param = parser.get<std::string>("lat");
    std::string angle_param = parser.get<std::string>("angle");
    std::string radius_param = parser.get<std::string>("radius");

    float longMin = std::stof(long_param.substr(0,long_param.find_first_of(":")).c_str());
    float longStep = std::stof(long_param.substr(long_param.find_first_of(":")+1,long_param.find_last_of(":")-1).c_str());
    float longMax = std::stof(long_param.substr(long_param.find_last_of(":")+1).c_str());

    float latMin = std::stof(lat_param.substr(0,lat_param.find_first_of(":")).c_str());
    float latStep = std::stof(lat_param.substr(lat_param.find_first_of(":")+1,lat_param.find_last_of(":")-1).c_str());
    float latMax = std::stof(lat_param.substr(lat_param.find_last_of(":")+1).c_str());

    float angleMin = std::stof(angle_param.substr(0,angle_param.find_first_of(":")).c_str());
    float angleStep = std::stof(angle_param.substr(angle_param.find_first_of(":")+1,angle_param.find_last_of(":")-1).c_str());
    float angleMax = std::stof(angle_param.substr(angle_param.find_last_of(":")+1).c_str());

    float radiusMin = std::stof(radius_param.substr(0,radius_param.find_first_of(":")).c_str());
    float radiusStep = std::stof(radius_param.substr(radius_param.find_first_of(":")+1,radius_param.find_last_of(":")-1).c_str());
    float radiusMax = std::stof(radius_param.substr(radius_param.find_last_of(":")+1).c_str());

    std::cout << "longMin: " << longMin << " ; longStep: " << longStep << " ; longMax: " << longMax << std::endl;
    std::cout << "latMin: " << latMin << " ; latStep: " << latStep << " ; latMax: " << latMax << std::endl;
    std::cout << "angleMin: " << angleMin << " ; angleStep: " << angleStep << " ; angleMax: " << angleMax << std::endl;
    std::cout << "radiusMin: " << radiusMin << " ; radiusStep: " << radiusStep << " ; radiusMax: " << radiusMax << std::endl;

    //Linemod parameters (sampling step at level 0 and 1 in the pyramid)
    int T_level0 = parser.get<int>("T_level0");
    int T_level1 = parser.get<int>("T_level1");

    std::cout << "Linemod, T_level0: " << T_level0 << std::endl;
    std::cout << "Linemod, T_level1: " << T_level1 << std::endl;

    //Render / camera parameters
    double renderWindowWidth = parser.get<double>("renderWindowWidth");
    double renderWindowHeight = parser.get<double>("renderWindowHeight");
    double fx = parser.get<double>("fx");
    double fy = parser.get<double>("fy");
    double cx = parser.get<double>("cx");
    double cy = parser.get<double>("cy");

    std::cout << "Rendering window width: " << renderWindowWidth << std::endl;
    std::cout << "Rendering window height: " << renderWindowHeight << std::endl;
    std::cout << "Camera parameters, fx: " << fx << std::endl;
    std::cout << "Camera parameters, fy: " << fy << std::endl;
    std::cout << "Camera parameters, cx: " << cx << std::endl;
    std::cout << "Camera parameters, cy: " << cy << std::endl;

    //Path to save the rendered views (for debugging or testing) if needed
    std::string saveTrainingImg = parser.get<std::string>("saveTrainingImg");

    std::cout << "Path to the directory to save the rendered images: " << saveTrainingImg << std::endl;

    ///Linemod training///
    CDFF::Common::LinemodTraining::LinemodBasedPoseDetector linemodDetector;
    if (useDepth)
    {
        linemodDetector.InitAs3D(T_level0, T_level1);
        std::cout << " *** 3D LineMOD initialized, T_level0: " << T_level0
                  << " ; T_level1: " << T_level1 << " ... " << std::endl;
    }
    else
    {
        linemodDetector.InitAs2D(T_level0, T_level1);
        std::cout << " *** 2D LineMOD initialized, T_level0: " << T_level0
                  << " ; T_level1: " << T_level1 << " ... " << std::endl;
    }

    std::cout << " *** Training ... "<<std::endl;
    int64 t0 = cv::getTickCount();

    //Camera intrinsic parameters
    linemodDetector._winW = renderWindowWidth;
    linemodDetector._winH = renderWindowHeight;
    linemodDetector._fx = fx;
    linemodDetector._fy = fy;
    linemodDetector._cx = cx;
    linemodDetector._cy = cy;
    linemodDetector._saveDir = saveTrainingImg;

    linemodDetector.Train(CADmodel, isPLY,
                           longMin, longMax, longStep,
                           latMin, latMax, latStep,
                           angleMin, angleMax, angleStep,
                           radiusMin, radiusMax, radiusStep);
    int64 t1 = cv::getTickCount();
    std::cout << " *** Done in "<< (t1-t0)/cv::getTickFrequency() << " seconds" << std::endl;

    linemodDetector.SaveTraining(CADmodel);
    std::cout << " *** Training templates saved in "
              << CADmodel.substr(0, CADmodel.find_last_of("."))
              << "(_training.dat/_poses.dat)" << std::endl;

    return 0;
}
/** @} */
