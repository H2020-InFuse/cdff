/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Triangulation class.
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
#include "Triangulation.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include "opencv2/calib3d.hpp"
#include <SupportTypes.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace SupportTypes;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Triangulation::Triangulation()
	{
	configurationFilePath = "";
	}

Triangulation::~Triangulation()
	{

	}

void Triangulation::configure()
	{

	}

void Triangulation::process() 
	{
	ValidateInputs(inCorrespondenceMap, inTransform);
	cv::Mat projectionMatrix = Convert(inTransform);
	cv::Mat pointsVectorAtSource = ConvertAtPose(inCorrespondenceMap, SOURCE_CAMERA);
	cv::Mat pointsVectorAtSink = ConvertAtPose(inCorrespondenceMap, SINK_CAMERA);
	cv::Mat uniformPointCloudMatrix = Triangulate(projectionMatrix, pointsVectorAtSource, pointsVectorAtSink);
	outPointCloud = Convert(uniformPointCloudMatrix);
	}

cv::Mat Triangulation::Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtSource, cv::Mat pointsVectorAtSink)
	{
	cv::Mat uniformPointCloudMatrix;
	cv::triangulatePoints(projectionMatrix, projectionMatrix, pointsVectorAtSource, pointsVectorAtSink, uniformPointCloudMatrix);
	return uniformPointCloudMatrix;
	}

cv::Mat Triangulation::Convert(Transform3DConstPtr transform)
	{
	Eigen::Quaternionf eigenRotation( GetWOrientation(*transform), GetXOrientation(*transform), GetYOrientation(*transform), GetZOrientation(*transform));
	Eigen::Matrix3f rotationMatrix = eigenRotation.toRotationMatrix();

	cv::Mat projectionMatrix(3, 4, CV_32FC1);
	for(unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			projectionMatrix.at<float>(rowIndex, columnIndex) = rotationMatrix(rowIndex, columnIndex);
			} 
		}
	projectionMatrix.at<float>(0, 3) = GetXPosition(*transform);
	projectionMatrix.at<float>(1, 3) = GetYPosition(*transform);
	projectionMatrix.at<float>(2, 3) = GetZPosition(*transform);	
	
	return projectionMatrix;
	}

cv::Mat Triangulation::ConvertAtPose(CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier)
	{
	cv::Mat pointsVector(2, GetNumberOfCorrespondences(*correspondenceMap), CV_32FC1);

	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D point;
		if (cameraPoseIdentifier == SOURCE_CAMERA)
			{
			point = GetSource(*correspondenceMap, correspondenceIndex);
			}
		else
			{
			point = GetSink(*correspondenceMap, correspondenceIndex);
			}
		pointsVector.at<float>(0, correspondenceIndex) = point.x;
		pointsVector.at<float>(1, correspondenceIndex) = point.y;		
		}

	return pointsVector;
	}

PointCloudConstPtr Triangulation::Convert(cv::Mat uniformPointCloudMatrix)
	{
	PointCloudPtr pointCloud = new PointCloud();
	ClearPoints(*pointCloud);

	for(unsigned pointIndex = 0; pointIndex < uniformPointCloudMatrix.cols; pointIndex++)
		{
		float uniformPointX = uniformPointCloudMatrix.at<float>(0, pointIndex);
		float uniformPointY = uniformPointCloudMatrix.at<float>(1, pointIndex);
		float uniformPointZ = uniformPointCloudMatrix.at<float>(2, pointIndex);
		float uniformPointFactor = uniformPointCloudMatrix.at<float>(3, pointIndex);
		AddPoint(*pointCloud, uniformPointX/uniformPointFactor, uniformPointY/uniformPointFactor, uniformPointZ/uniformPointFactor);
		}
	return pointCloud;
	}

void Triangulation::ValidateInputs(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, PoseWrapper::Transform3DConstPtr transform)
	{

	}


}


/** @} */
