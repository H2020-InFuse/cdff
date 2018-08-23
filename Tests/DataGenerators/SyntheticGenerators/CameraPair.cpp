/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CameraPair.cpp
 * @date 06/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the CameraPair class.
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
#include "CameraPair.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>
#include <BaseTypes.hpp>
#include <random>

using namespace CorrespondenceMap2DWrapper;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CameraPair::CameraPair()
	{
	SetFirstCameraMatrix( cv::Point2d(1, 1), cv::Point2d(0,0) );
	SetSecondCameraMatrix( cv::Point2d(1, 1), cv::Point2d(0,0) );
	SetSecondCameraRotationAroundZ(0);
	SetSecondCameraTranslation(0,0,0);
	changed = true;
	std::srand(std::time(NULL));
	}

CameraPair::~CameraPair()
	{

	}

void CameraPair::SetFirstCameraMatrix(cv::Point2d focalLength, cv::Point2d principlePoint)
	{
	firstCameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	firstCameraMatrix.at<float>(0,0) = focalLength.x;
	firstCameraMatrix.at<float>(1,1) = focalLength.y;
	firstCameraMatrix.at<float>(0,2) = principlePoint.x;
	firstCameraMatrix.at<float>(1,2) = principlePoint.y;
	firstCameraMatrix.at<float>(2,2) = 1;
	changed = true;
	}

void CameraPair::SetSecondCameraMatrix(cv::Point2d focalLength, cv::Point2d principlePoint)
	{
	secondCameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	secondCameraMatrix.at<float>(0,0) = focalLength.x;
	secondCameraMatrix.at<float>(1,1) = focalLength.y;
	secondCameraMatrix.at<float>(0,2) = principlePoint.x;
	secondCameraMatrix.at<float>(1,2) = principlePoint.y;
	secondCameraMatrix.at<float>(2,2) = 1;
	changed = true;
	}

void CameraPair::SetSecondCameraRotationAroundX(float angle)
	{
	secondCameraRotation = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	secondCameraRotation.at<float>(1,1) = std::cos(angle);
	secondCameraRotation.at<float>(1,2) = -std::sin(angle);
	secondCameraRotation.at<float>(2,1) = std::sin(angle);
	secondCameraRotation.at<float>(2,2) = std::cos(angle);
	secondCameraRotation.at<float>(0,0) = 1;
	changed = true;
	}

void CameraPair::SetSecondCameraRotationAroundY(float angle)
	{
	secondCameraRotation = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	secondCameraRotation.at<float>(0,0) = std::cos(angle);
	secondCameraRotation.at<float>(2,0) = -std::sin(angle);
	secondCameraRotation.at<float>(0,2) = std::sin(angle);
	secondCameraRotation.at<float>(2,2) = std::cos(angle);
	secondCameraRotation.at<float>(1,1) = 1;
	changed = true;
	}

void CameraPair::SetSecondCameraRotationAroundZ(float angle)
	{
	secondCameraRotation = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	secondCameraRotation.at<float>(0,0) = std::cos(angle);
	secondCameraRotation.at<float>(0,1) = -std::sin(angle);
	secondCameraRotation.at<float>(1,0) = std::sin(angle);
	secondCameraRotation.at<float>(1,1) = std::cos(angle);
	secondCameraRotation.at<float>(2,2) = 1;
	changed = true;
	}

void CameraPair::SetSecondCameraTranslation(float x, float y, float z)
	{
	secondCameraTranslation = cv::Mat(1, 3, CV_32FC1, cv::Scalar(0));
	secondCameraTranslation.at<float>(0,0) = x;
	secondCameraTranslation.at<float>(0,1) = y;
	secondCameraTranslation.at<float>(0,2) = z;
	changed = true;
	}

void CameraPair::SetSecondCameraPose(Pose3DConstPtr secondCameraPose)
	{
	secondCameraTranslation = cv::Mat(1, 3, CV_32FC1, cv::Scalar(0));
	secondCameraTranslation.at<float>(0,0) = GetXTranslation(*secondCameraPose);
	secondCameraTranslation.at<float>(0,1) = GetYTranslation(*secondCameraPose);
	secondCameraTranslation.at<float>(0,2) = GetZTranslation(*secondCameraPose);

	float qx = GetXOrientation(*secondCameraPose);
	float qy = GetYOrientation(*secondCameraPose);
	float qz = GetZOrientation(*secondCameraPose);
	float qw = GetWOrientation(*secondCameraPose);

	secondCameraRotation = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	secondCameraRotation.at<float>(0,0) = 1 - 2*qy*qy - 2*qz*qz;
	secondCameraRotation.at<float>(0,1) = 2*qx*qy - 2*qz*qw;
	secondCameraRotation.at<float>(0,2) = 2*qx*qz + 2*qy*qw;
	secondCameraRotation.at<float>(1,0) = 2*qx*qy + 2*qz*qw;
	secondCameraRotation.at<float>(1,1) = 1 - 2*qx*qx - 2*qz*qz;
	secondCameraRotation.at<float>(1,2) = 2*qy*qz - 2*qx*qw;
	secondCameraRotation.at<float>(2,0) = 2*qx*qz - 2*qy*qw;
	secondCameraRotation.at<float>(2,1) = 2*qy*qz + 2*qx*qw;
	secondCameraRotation.at<float>(2,2) = 1 - 2*qx*qx - 2*qy*qy;
	changed = true;
	}

cv::Mat CameraPair::GetFundamentalMatrix()
	{	
	PrepareCameraPair();
	return fundamentalMatrix;
	}

CorrespondenceMap2DConstPtr CameraPair::GetSomeRandomCorrespondences(unsigned correspondencesNumber, float noiseStandardDeviation)
	{
	cv::Mat pointCloud;
	return GetSomeRandomCorrespondences(correspondencesNumber, pointCloud, noiseStandardDeviation);
	}

CorrespondenceMap2DConstPtr CameraPair::GetSomeRandomCorrespondences(unsigned correspondencesNumber, cv::Mat& pointCloud, float noiseStandardDeviation)
	{
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, noiseStandardDeviation);

	const double EPSILON = 0.001;
	PrepareCameraPair();
	
	pointCloud = cv::Mat(3, correspondencesNumber, CV_32FC1);

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < correspondencesNumber; index++)
		{		
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(3,0) = 1;
		pointCloud.at<float>(0, index) = point3d.at<float>(0,0);
		pointCloud.at<float>(1, index) = point3d.at<float>(1,0);
		pointCloud.at<float>(2, index) = point3d.at<float>(2,0);
		
		cv::Mat firstPoint2d = firstProjectionMatrix * point3d;
		cv::Mat secondPoint2d = secondProjectionMatrix * point3d;

		cv::Mat test = secondPoint2d.t() * fundamentalMatrix * firstPoint2d;
		ASSERT ( std::abs( test.at<float>(0,0) ) < EPSILON, "Fundamental Matrix is wrong");

		Point2D source, sink;

		source.x = firstPoint2d.at<float>(0, 0) / firstPoint2d.at<float>(2, 0) + distribution(generator);
		source.y = firstPoint2d.at<float>(1, 0) / firstPoint2d.at<float>(2, 0) + distribution(generator);
		sink.x = secondPoint2d.at<float>(0, 0) / secondPoint2d.at<float>(2, 0) + distribution(generator);
		sink.y = secondPoint2d.at<float>(1, 0) / secondPoint2d.at<float>(2, 0) + distribution(generator);

		AddCorrespondence(*input, source, sink, 1);
		}
	return input;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void CameraPair::ComputeProjectionMatrices()
	{
	cv::Mat identityMatrix = cv::Mat(3, 4, CV_32FC1, cv::Scalar(0));
	identityMatrix.at<float>(0,0) = 1;
	identityMatrix.at<float>(1,1) = 1;
	identityMatrix.at<float>(2,2) = 1;
	firstProjectionMatrix = firstCameraMatrix * identityMatrix;
	
	cv::Mat secondCameraRotoTranslation;
	cv::hconcat(secondCameraRotation, secondCameraTranslation.t(), secondCameraRotoTranslation);
	secondProjectionMatrix = secondCameraMatrix * secondCameraRotoTranslation;
	}

void CameraPair::ComputeFundamentalMatrix()
	{
	cv::Mat firstCameraCentre(4, 1, CV_32FC1, cv::Scalar(0));
	firstCameraCentre.at<float>(3, 0) = 1;

	cv::Mat secondEpipole = secondProjectionMatrix * firstCameraCentre;
	cv::Mat secondEpipoleSkew(3, 3, CV_32FC1, cv::Scalar(0));
	secondEpipoleSkew.at<float>(0,1) = - secondEpipole.at<float>(2,0);
	secondEpipoleSkew.at<float>(0,2) = + secondEpipole.at<float>(1,0);
	secondEpipoleSkew.at<float>(1,0) = + secondEpipole.at<float>(2,0);
	secondEpipoleSkew.at<float>(1,2) = - secondEpipole.at<float>(0,0);
	secondEpipoleSkew.at<float>(2,0) = - secondEpipole.at<float>(1,0);
	secondEpipoleSkew.at<float>(2,1) = + secondEpipole.at<float>(0,0);
			
	cv::Mat firstProjectionMatrixPseudoinverse(4, 3, CV_32FC1);
	cv::invert(firstProjectionMatrix, firstProjectionMatrixPseudoinverse, cv::DECOMP_SVD);

	fundamentalMatrix = secondEpipoleSkew * secondProjectionMatrix * firstProjectionMatrixPseudoinverse;
	}

void CameraPair::PrepareCameraPair()
	{
	if (!changed)
		{
		return;
		}

	ComputeProjectionMatrices();
	ComputeFundamentalMatrix();

	changed = false;
	}

}
/** @} */
