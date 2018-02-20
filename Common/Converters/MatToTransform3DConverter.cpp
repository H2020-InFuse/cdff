/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToTransform3DConverter.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of MatToTransform3DConverter.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */

#include "MatToTransform3DConverter.hpp"
#include <Errors/Assert.hpp>
#include <Eigen/Geometry>

namespace Converters {

using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const PoseWrapper::Transform3DConstPtr MatToTransform3DConverter::Convert(const cv::Mat transform)
	{
	ASSERT(transform.rows >= 3 && transform.cols >= 4, "MatToTransform3DConverter Error, an invalid matrix was passed");
	ASSERT(transform.type() == CV_32FC1 || transform.type() == CV_64FC1, "MatToTransform3DConverter Error, an invalid matrix type was passed");

	Transform3DPtr conversion = new Transform3D();
	Eigen::Matrix3f eigenRotationMatrix;
	if (transform.type() == CV_32FC1)
		{
		SetPosition(*conversion, transform.at<float>(0, 3), transform.at<float>(1, 3), transform.at<float>(2, 3) );

		eigenRotationMatrix <<  transform.at<float>(0, 0), transform.at<float>(0, 1), transform.at<float>(0, 2),
					transform.at<float>(1, 0), transform.at<float>(1, 1), transform.at<float>(1, 2),
					transform.at<float>(2, 0), transform.at<float>(2, 1), transform.at<float>(2, 2);
		}
	else
		{
		SetPosition(*conversion, transform.at<double>(0, 3), transform.at<double>(1, 3), transform.at<double>(2, 3) );

		eigenRotationMatrix <<  transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
					transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
					transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2);
		}
	
	ASSERT_CLOSE(std::abs(eigenRotationMatrix.determinant()), 1.00, 0.00001, "EigenTransformToTransform3DConverter, An invalid rotation was passed during conversion.")
	Eigen::Quaternionf eigenRotation (eigenRotationMatrix);
	SetOrientation(*conversion, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return conversion;
	}

const PoseWrapper::Transform3DSharedConstPtr MatToTransform3DConverter::ConvertShared(const cv::Mat transform)
	{
	Transform3DConstPtr conversion = Convert(transform);
	Transform3DSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	} 

}

/** @} */
