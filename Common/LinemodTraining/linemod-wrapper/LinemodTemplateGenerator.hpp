/**
 * @addtogroup LinemodTraining
 * @{
 */

#ifndef LINEMODTRAINING_LINEMODTEMPLATEGENERATOR_HPP
#define LINEMODTRAINING_LINEMODTEMPLATEGENERATOR_HPP

#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkJPEGReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkOBJImporter.h>

#include <opencv2/opencv.hpp>

namespace CDFF
{
namespace Common
{
namespace LinemodTraining
{

/*!
 * @brief This class implements a template generator used in LINEMOD training.<br>
 * This generator use a 3D model and a pose to generate an image suitable for LINEMOD training<br>
 */
class LinemodTemplateGenerator
{
public:

    /*!
     * @brief Constructor from 3D model and camera parameters.
     * @param rstr_specificDatasetNameWithMesh specific dataset name which contains mesh else all datasets renderable are used
     * @param mesh_from_ply true if .ply format, otherwise .obj is assumed
     * @param winW width of the rendering window
     * @param winH height of the rendering window
     * @param fx camera focal length in x (not used, only fy is used to set the camera view angle)
     * @param fy camera focal length in y
     * @param cx camera principal point in x
     * @param cy camera principal point in y
     */
    LinemodTemplateGenerator(const std::string& rstr_specificDatasetNameWithMesh, bool mesh_from_ply,
                             double winW, double winH, double fx, double fy, double cx, double cy);

    /*!
     * @brief Destructor
     * Free internal objects such as rendering contexts and shader objects.
     */
    virtual ~LinemodTemplateGenerator();

    /*!
     * @brief set current camera pose
     * The given matrix is interpreted as a camera frame to world frame transformation.
     * @param campos the 4x4 matrix representing the pose
     */
    void setPose(const cv::Affine3d& campos);

    /*!
    * @brief get the current camera pose as a camera frame to world frame transformation
    * @return a cv::Affine3d matrix representing the current camera pose
    */
    const cv::Affine3d &getPose() const;

    /*!
     * @brief set the 3D model
     * If not set in the constructor the 3D model <b>must</b> be set with this method.
     * @param rstr_speficicDatasetNameWithMesh object model without extension
     */
    void setModel(const std::string &rstr_specificdatasetNameWithMesh);

    /**
     * @brief do a rendering
     */
    void render(int num_modalities);

    /**
     * @brief convert a Matx44d to an OpenCV 3x3 matrix and a translation vector assuming the Matx44d is a rigid transform
     * @param rm4_M the input 4x4 matrix of the rigid transform
     * @param R the output 3x3 rotation matrix
     * @param T the output translation vector
     */
    static void matrix4_to_RT(const cv::Affine3d &rm4_M, cv::Matx33d &R, cv::Vec3d &T);
    /**
     * @brief convert a Matx44d to an OpenCV rotation and translation vectors assuming the Matx44d is a rigid transform
     * @param rm4_M the input 4x4 matrix of the rigid transform
     * @param R the output rotation vector
     * @param T the output translation vector
     */
    static void matrix4_to_RT(const cv::Affine3d &rm4_M, cv::Vec3d &R, cv::Vec3d &T);
    /**
     * @brief convert an OpenCV 3x3 matrix and a translation vector to a  OpenCV 4x4
     * @param R the input 3x3 rotation matrix
     * @param T the input translation vector
     * @param rm4_M the output 4x4 matrix
     */
    static void RT_to_matrix4(const cv::Matx33d &R, const cv::Vec3d &T, cv::Affine3d &rm4_M);
    /**
     * @brief convert a pair of OpenCV rotation and translation vectors to a Matrix4
     * @param R the input rotation vector
     * @param T the input translation vector
     * @param rm4_M the output 4x4 matrix
     */
    static void RT_to_matrix4(const cv::Vec3d &R, const cv::Vec3d &T, cv::Affine3d &rm4_M);

    /// @brief object model name
    std::string _str_specificDatasetNameWithMesh;

    /// @brief the current camera pose
    cv::Affine3d _pose;

    /// @brief color image buffer
    cv::Mat _x_colorImage;

    /// @brief image buffer to read back position/depth data
    cv::Mat _x_depthImage;

    /// @brief image mask
    cv::Mat _x_maskImage;

    /// @brief PLY/OBJ Reader/mapper/actor
    bool fromPLY;
    vtkSmartPointer<vtkAbstractPolyDataReader> _px_reader;
    vtkSmartPointer<vtkPolyDataMapper> _px_mapper;
    vtkSmartPointer<vtkActor> _px_actor;
    vtkSmartPointer<vtkOBJImporter> _px_obj;

    /// @brief Camera
    vtkSmartPointer<vtkCamera> _px_camera;
    double near_clipping, far_clipping;

    /// @brief Render
    vtkSmartPointer<vtkRenderer> _px_renderer;
    vtkSmartPointer<vtkRenderWindow> _px_renderWindow;
};

}
}
}
#endif // LINEMODTRAINING_LINEMODTEMPLATEGENERATOR_HPP

/** @} */
