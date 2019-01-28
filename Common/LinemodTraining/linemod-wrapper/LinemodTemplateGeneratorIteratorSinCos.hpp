/**
 * @addtogroup LinemodTraining
 * @{
 */

#ifndef LINEMODTRAINING_LINEMODTEMPLATEGENERATORITERATORSINCOS_HPP
#define LINEMODTRAINING_LINEMODTEMPLATEGENERATORITERATORSINCOS_HPP

#include <string>
#include <opencv2/core/core.hpp>

#include "LinemodTemplateGenerator.hpp"

namespace CDFF
{
namespace Common
{
namespace LinemodTraining
{

/*!
 * @brief This class implements a pseudo iterator of the viewpoint over a sphere.<br>
 * It is used for LINEMOD training<br>
 */
class LinemodTemplateGeneratorIteratorSinCos
{
private:

    /** The renderer object containing the scene and that will render images */
    LinemodTemplateGenerator* _px_renderer;

    /** Current, start, end and increment values for longitude, latitude on the unit sphere */
    float _f_lonMin, _f_lonMax, _f_lonStep, _f_lon;
    float _f_latMin, _f_latMax, _f_latStep, _f_lat;
    /** Current, start, end and increment values for 3rd axis rotation angle */
    float _f_angleMin, _f_angleMax, _f_angleStep, _f_angle;
    /** Current, start, end and increment values for the scale sampling */
    float _f_radiusMin, _f_radiusMax, _f_radiusStep, _f_radius;

    /** Current pose matrix */
    cv::Affine3d _mat4_currentPose;
    /** size of iterator */
    int _i_size;
    /** iterator end flag */
    bool _b_isDone;

public:
    LinemodTemplateGeneratorIteratorSinCos(LinemodTemplateGenerator* px_renderer,
                                           float f_lonMin, float f_lonMax, float f_lonStep,
                                           float f_latMin, float f_latMax, float f_latStep,
                                           float f_angleMin, float f_angleMax, float f_angleStep,
                                           float f_radiusMin, float f_radiusMax, float f_radiusStep);

    /** Iterate to get to a different view
     * We don't implement the postfix operator on purpose
     * @return an incremented version of itself
     */
    LinemodTemplateGeneratorIteratorSinCos& operator++();

    /**
     * @return true if we are done with all the views, false otherwise
     */
    bool isDone() const
    {
        return (_b_isDone);
    }

    void render(std::vector<cv::Mat> &sources, cv::Mat &mask_out, int num_modalities);

    /*!
     * \brief getPoseMatrix4
     * \return the current pose
     */
    inline cv::Affine3d getPoseMatrix4() {
        _px_renderer->getPose();
        return _mat4_currentPose;
    }

    /*!
     * \brief getSize
     * \return the current size
     */
    inline int getSize() { return _i_size; }
};

}
}
}
#endif //LINEMODTRAINING_LINEMODTEMPLATEGENERATORITERATORSINCOS_HPP

/** @} */
