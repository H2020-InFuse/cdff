/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DToMatConverter.cpp
 * @date 20/02/2018
 * @authors Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of Transform3DToMatConverter class.
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

#include "Transform3DToMatConverter.hpp"
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
const cv::Mat Transform3DToMatConverter::Convert(const PoseWrapper::Transform3DConstPtr& transform)
	{
	Eigen::Matrix4f conversion;

	Eigen::Quaternionf eigenRotation( GetWOrientation(*transform), GetXOrientation(*transform), GetYOrientation(*transform), GetZOrientation(*transform));
	Eigen::Matrix3f rotationMatrix = eigenRotation.toRotationMatrix();

	Eigen::Translation<float, 3> eigenTranslation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));	
	
	conversion << 	rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), eigenTranslation.x(),
			rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), eigenTranslation.y(),
			rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), eigenTranslation.z(),
			0, 0, 0, 1;

	cv::Mat matrixConversion(3, 4, CV_32FC1);
	for(unsigned row = 0; row < 3; row++)
		{
		for(unsigned column = 0; column<4; column++)
			{
			matrixConversion.at<float>(row, column) = conversion(row, column);
			}
		}

	return matrixConversion;
	}

const cv::Mat Transform3DToMatConverter::ConvertShared(const PoseWrapper::Transform3DSharedConstPtr& transform)
	{
	return Convert(transform.get());
	}

}

/** @} */
