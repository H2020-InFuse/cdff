/**
 * @addtogroup LinemodTraining
 * @{
 */

#include <cmath>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "LinemodTemplateGeneratorIteratorSinCos.hpp"

namespace LinemodTraining
{

LinemodTemplateGeneratorIteratorSinCos::LinemodTemplateGeneratorIteratorSinCos(LinemodTemplateGenerator *px_renderer,
                                                                               float f_lonMin, float f_lonMax, float f_lonStep,
                                                                               float f_latMin, float f_latMax, float f_latStep,
                                                                               float f_angleMin, float f_angleMax, float f_angleStep,
                                                                               float f_radiusMin, float f_radiusMax, float f_radiusStep)
    :
      _px_renderer(px_renderer),

      _f_lonMin(f_lonMin * static_cast<float>(CV_PI/180.0)),
      _f_lonMax(f_lonMax * static_cast<float>(CV_PI/180.0)),
      _f_lonStep(f_lonStep * static_cast<float>(CV_PI/180.0)),
      _f_lon(f_lonMin * static_cast<float>(CV_PI/180.0)),

      _f_latMin(f_latMin * static_cast<float>(CV_PI/180.0)),
      _f_latMax(f_latMax * static_cast<float>(CV_PI/180.0)),
      _f_latStep(f_latStep * static_cast<float>(CV_PI/180.0)),
      _f_lat(f_latMin * static_cast<float>(CV_PI/180.0)),

      _f_angleMin(f_angleMin * static_cast<float>(CV_PI/180.0)),
      _f_angleMax(f_angleMax * static_cast<float>(CV_PI/180.0)),
      _f_angleStep(f_angleStep * static_cast<float>(CV_PI/180.0)),
      _f_angle(f_angleMin * static_cast<float>(CV_PI/180.0)),

      _f_radiusMin(f_radiusMin),
      _f_radiusMax(f_radiusMax),
      _f_radiusStep(f_radiusStep),
      _f_radius(f_radiusMin),
      _mat4_currentPose(),
      _i_size( static_cast<int>((std::floor((f_lonMax-f_lonMin)/f_lonStep)+1)
               * (std::floor((f_latMax-f_latMin)/f_latStep)+1)
               * (std::floor((f_angleMax-f_angleMin)/f_angleStep)+1)
               * (std::floor((f_radiusMax-f_radiusMin)/f_radiusStep)+1))),
      _b_isDone(false)
{
}

LinemodTemplateGeneratorIteratorSinCos & LinemodTemplateGeneratorIteratorSinCos::operator++()
{
    _f_lon += _f_lonStep ;
    if (_f_lon > _f_lonMax){
        _f_lon = _f_lonMin;
        _f_lat += _f_latStep;
        if (_f_lat > _f_latMax){
            _f_lat = _f_latMin;
            _f_angle += _f_angleStep;
            if (_f_angle > _f_angleMax)
            {
                _f_angle = _f_angleMin;
                _f_radius += _f_radiusStep;
                if (_f_radius > _f_radiusMax)
                {
                    _b_isDone = true;
                }
            }
        }
    }
    return *this;
}

void LinemodTemplateGeneratorIteratorSinCos::render(std::vector<cv::Mat> &sources, cv::Mat &mask_out, int num_modalities)
{
    if (isDone())
        return;

    // North east down (NED) Frame
    cv::Mat R_NED = (cv::Mat_<double>(3,3) <<
                     -sin( static_cast<double>(_f_lat))*cos( static_cast<double>(_f_lon)),   -sin( static_cast<double>(_f_lon)),      -cos( static_cast<double>(_f_lat))*cos( static_cast<double>(_f_lon)),
                     -sin( static_cast<double>(_f_lat))*sin( static_cast<double>(_f_lon)),    cos( static_cast<double>(_f_lon)),      -cos( static_cast<double>(_f_lat))*sin( static_cast<double>(_f_lon)),
                     cos( static_cast<double>(_f_lat)),              0,                  -sin( static_cast<double>(_f_lat)));

    // Rotation around Z
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
                   cos( static_cast<double>(_f_angle)),    -sin( static_cast<double>(_f_angle)),      0,
                   sin( static_cast<double>(_f_angle)),    cos( static_cast<double>(_f_angle)),       0,
                   0,               0,                  1);

    cv::Mat Position = R_NED*R_z*cv::Mat(cv::Vec3d(0,0,- static_cast<double>(_f_radius)));
    _mat4_currentPose = cv::Affine3d(R_NED*R_z, cv::Vec3d(Position));

    _px_renderer->setPose(_mat4_currentPose);
    _px_renderer->render(num_modalities);

    sources.push_back(_px_renderer->_x_colorImage);
    if(num_modalities == 2)
        sources.push_back(_px_renderer->_x_depthImage);
    mask_out = _px_renderer->_x_maskImage;
}

}

/** @} */
