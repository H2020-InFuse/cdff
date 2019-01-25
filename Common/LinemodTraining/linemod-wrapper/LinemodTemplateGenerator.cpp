/**
 * @addtogroup LinemodTraining
 * @{
 */

#include <GL/glut.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);

#include <vtksys/SystemTools.hxx>

#include <opencv2/opencv.hpp>
#include <sys/time.h>

#include "LinemodTemplateGenerator.hpp"

namespace LinemodTraining
{

LinemodTemplateGenerator::LinemodTemplateGenerator(const std::string &rstr_specificDatasetNameWithMesh, bool mesh_from_ply,
                                                   double winW, double winH, double /*fx*/, double fy, double cx, double cy) :
    fromPLY(mesh_from_ply)
{
    if(fromPLY){
        _px_reader = vtkSmartPointer<vtkPLYReader>::New();
    }else{
        _px_reader = vtkSmartPointer<vtkOBJReader>::New();
    }
    _px_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    _px_actor = vtkSmartPointer<vtkActor>::New();
    _px_obj = vtkSmartPointer<vtkOBJImporter>::New();

    _px_camera = vtkSmartPointer<vtkCamera>::New();
    //Focal length to view angle
    double fov = 2*atan( winH / (2*fy) );
    _px_camera->SetViewAngle(fov*180/M_PI);
    // convert the principal point to window center (normalized coordinate system) and set it
    double wcx = -2.0*(cx - winW/2.0) / winW;
    double wcy =  2.0*(cy - winH/2.0) / winH;
    _px_camera->SetWindowCenter(wcx, wcy);
    _px_camera->SetFocalPoint(0, 0, 0);
    _px_camera->GetClippingRange (near_clipping, far_clipping);

    _px_renderer = vtkSmartPointer<vtkRenderer>::New();
    _px_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

    _px_renderWindow->AddRenderer(_px_renderer);
    _px_renderWindow->SetSize(winW, winH);

    _px_renderer->SetActiveCamera(_px_camera);
    _px_renderer->SetBackground(0.1804,0.5451,0.3412); // Sea green

    setModel(rstr_specificDatasetNameWithMesh);
}

LinemodTemplateGenerator::~LinemodTemplateGenerator()
{
}

void LinemodTemplateGenerator::setModel(const std::string& rstr_specificdatasetNameWithMesh)
{
    _str_specificDatasetNameWithMesh = rstr_specificdatasetNameWithMesh;

    if(fromPLY){
        _px_reader->SetFileName ( std::string(_str_specificDatasetNameWithMesh + ".ply").c_str() );
        _px_mapper->SetInputConnection(_px_reader->GetOutputPort());
        _px_actor->SetMapper(_px_mapper);

        _px_renderer->AddActor(_px_actor);
    }else{
        std::string objPath = std::string(_str_specificDatasetNameWithMesh + ".obj");
        _px_obj->SetFileName(objPath.c_str());
        _px_obj->SetFileNameMTL(std::string(_str_specificDatasetNameWithMesh + ".mtl").c_str());
        std::string texturePath = vtksys::SystemTools::GetFilenamePath(objPath);
        _px_obj->SetTexturePath(texturePath.c_str());
        _px_obj->SetRenderWindow(_px_renderWindow);
        _px_obj->Update();
    }
}

void LinemodTemplateGenerator::setPose(const cv::Affine3d& campos)
{
    _pose = campos;

    // Position = Center of Frame
    cv::Vec3d position = campos.translation();
    _px_camera->SetPosition(position[0], position[1], position[2]);

    // yDir = campos*(1,0,0) (yDir is North in NED frame)
    _px_camera->SetViewUp(campos.matrix(0,0),campos.matrix(1,0),campos.matrix(2,0));
}

const cv::Affine3d &LinemodTemplateGenerator::getPose() const {
    return _pose;
}

void LinemodTemplateGenerator::render(int num_modalities)
{
    _px_renderWindow->Render();

    vtkSmartPointer<vtkWindowToImageFilter> windowToRGBFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToRGBFilter->SetInput(_px_renderWindow);
    windowToRGBFilter->Update();

    vtkImageData* RGB = windowToRGBFilter->GetOutput();

    // Construct the OpenCv Mat
    int dims[3];
    RGB->GetDimensions(dims);
    cv::Mat RGB_cv(dims[1], dims[0], CV_8UC3, RGB->GetScalarPointer()); // Unsigned int, 4 channels
    cv::cvtColor(RGB_cv, _x_colorImage, cv::COLOR_BGR2RGB);
    cv::flip(_x_colorImage,_x_colorImage, 0);

    vtkSmartPointer<vtkWindowToImageFilter> windowToDepthFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToDepthFilter->SetInput(_px_renderWindow);
    windowToDepthFilter->SetInputBufferTypeToZBuffer();
    windowToDepthFilter->Update();

    vtkImageData* DEPTH = windowToDepthFilter->GetOutput();

    // Construct the OpenCv Mat
    DEPTH->GetDimensions(dims);
    cv::Mat DEPTH_cv(dims[1], dims[0], CV_32FC1, DEPTH->GetScalarPointer());
    cv::flip(DEPTH_cv,DEPTH_cv, 0);
    _x_maskImage = DEPTH_cv < 0.999999;

    if(num_modalities == 2)
    {
        cv::Mat _x_depthImage_32F = (far_clipping*near_clipping)/(far_clipping-(far_clipping-near_clipping)*DEPTH_cv)*1000;
        cv::multiply(_x_depthImage_32F, _x_maskImage/255, _x_depthImage, 1, CV_16UC1);
    }
}

void LinemodTemplateGenerator::matrix4_to_RT(const cv::Affine3d &rm4_M, cv::Matx33d &R, cv::Vec3d &T){
    R = rm4_M.rotation();
    T = rm4_M.translation();
}

void LinemodTemplateGenerator::matrix4_to_RT(const cv::Affine3d &rm4_M, cv::Vec3d &R, cv::Vec3d &T){
    R = rm4_M.rvec();
    T = rm4_M.translation();
}

void LinemodTemplateGenerator::RT_to_matrix4(const cv::Matx33d &R, const cv::Vec3d &T, cv::Affine3d &rm4_M){
    rm4_M = cv::Affine3d(R,T);
}

void LinemodTemplateGenerator::RT_to_matrix4(const cv::Vec3d &R, const cv::Vec3d &T, cv::Affine3d &rm4_M){
    rm4_M = cv::Affine3d(R,T);
}

}

/** @} */
