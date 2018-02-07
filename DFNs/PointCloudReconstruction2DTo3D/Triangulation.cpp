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
using namespace BaseTypesWrapper;
using namespace MatrixWrapper;

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
	ValidateInputs(inCorrespondenceMap, inFundamentalMatrix);
	cv::Mat projectionMatrix = ComputeSecondProjectionMatrix(inFundamentalMatrix, inSecondEpipole);
	cv::Mat pointsVectorAtSource = ConvertAtPose(inCorrespondenceMap, SOURCE_CAMERA);
	cv::Mat pointsVectorAtSink = ConvertAtPose(inCorrespondenceMap, SINK_CAMERA);
	cv::Mat uniformPointCloudMatrix = Triangulate(projectionMatrix, pointsVectorAtSource, pointsVectorAtSink);
	outPointCloud = Convert(uniformPointCloudMatrix);
	}

cv::Mat Triangulation::Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtSource, cv::Mat pointsVectorAtSink)
	{
	//The first camera projection matrix is [I|0] according to Result 9.14 of Multiple view geometry in computer vision. Richard Hartley, Andrew Zisserman.
	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0) );
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat homogeneousPointCloudMatrix;
	cv::triangulatePoints(identityProjection, projectionMatrix, pointsVectorAtSource, pointsVectorAtSink, homogeneousPointCloudMatrix);
	return homogeneousPointCloudMatrix;
	}

cv::Mat Triangulation::ComputeSecondProjectionMatrix(Matrix3dConstPtr fundamentalMatrix, Point2DConstPtr secondEpipole)
	{
	//Computation of the second projection matrix according to Result 9.14 of Multiple view geometry in computer vision. Richard Hartley, Andrew Zisserman.
	cv::Mat cvFundamentalMatrix(3, 3, CV_64FC1);
	for(unsigned row = 0; row < 3; row++)
		{
		for (unsigned column = 0; column < 3; column++)
			{
			cvFundamentalMatrix.at<double>(row, column) = GetElement(*fundamentalMatrix, row, column);
			}
		}
	
	cv::Mat epipoleMatrix(3, 3, CV_64FC1, cv::Scalar(0));
	epipoleMatrix.at<double>(0,1) = 1;
	epipoleMatrix.at<double>(0,2) = secondEpipole->y;
	epipoleMatrix.at<double>(1,0) = 1;
	epipoleMatrix.at<double>(1,2) = -secondEpipole->x;
	epipoleMatrix.at<double>(2,0) = -secondEpipole->y;
	epipoleMatrix.at<double>(2,1) = +secondEpipole->x;

	cv::Mat epipoleFundametalMatrixProduct = epipoleMatrix * cvFundamentalMatrix;

	cv::Mat projectionMatrix(3, 4, CV_32FC1, cv::Scalar(0));
	epipoleFundametalMatrixProduct.convertTo( projectionMatrix(cv::Rect(0,0,3,3)), CV_32FC1 );
	projectionMatrix.at<float>(0,3) = static_cast<float>(secondEpipole->x);
	projectionMatrix.at<float>(1,3) = static_cast<float>(secondEpipole->y);
	projectionMatrix.at<float>(2,3) = static_cast<float>(1);

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

PointCloudConstPtr Triangulation::Convert(cv::Mat homogeneousPointCloudMatrix)
	{
	PointCloudPtr pointCloud = new PointCloud();
	ClearPoints(*pointCloud);

	for(unsigned pointIndex = 0; pointIndex < homogeneousPointCloudMatrix.cols; pointIndex++)
		{
		float homogeneousPointX = homogeneousPointCloudMatrix.at<float>(0, pointIndex);
		float homogeneousPointY = homogeneousPointCloudMatrix.at<float>(1, pointIndex);
		float homogeneousPointZ = homogeneousPointCloudMatrix.at<float>(2, pointIndex);
		float homogeneousPointFactor = homogeneousPointCloudMatrix.at<float>(3, pointIndex);
		if (homogeneousPointFactor == 0)
			{
			AddPoint(*pointCloud, 0, 0, 0);
			}
		else
			{
			AddPoint(*pointCloud, homogeneousPointX/homogeneousPointFactor, homogeneousPointY/homogeneousPointFactor, homogeneousPointZ/homogeneousPointFactor);
			}
		}
	return pointCloud;
	}

void Triangulation::ValidateInputs(CorrespondenceMap2DConstPtr correspondenceMap, Matrix3dConstPtr fundamentalMatrix)
	{

	}


}


/** @} */
