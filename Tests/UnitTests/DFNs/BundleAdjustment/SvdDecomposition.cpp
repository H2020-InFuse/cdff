/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SvdDecomposition.cpp
 * @date 06/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN implementation SvdDecomposition.
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
#include <catch.hpp>
#include <BundleAdjustment/SvdDecomposition.hpp>
#include <MatToFrameConverter.hpp>
#include <Eigen/Dense>

using namespace dfn_ci;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (SVD Decomposition) and fail", "[failprocess]" )
{
	//Initialize Inputs
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = NewCorrespondenceMaps2DSequence(); 
	for(int index = 0; index < 6; index++)
		{
		CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
		AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
		}

	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Send input data to DFN
	svd->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	svd->process();

	// Query output data from DFN
	const Poses3DSequence& output = svd->posesSequenceOutput();
	bool success = svd->successOutput();

	REQUIRE(!success);

	// Cleanup
	delete(svd);
}

TEST_CASE( "Call to configure (SVD Decomposition)", "[configure]" )
{
	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Setup DFN
	svd->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/SvdDecomposition_Conf1.yaml");
	svd->configure();

	// Cleanup
	delete(svd);
}

#define EPSILON 0.0001
#define CLOSE(a, b) (b > a - EPSILON && b < a + EPSILON)
#define CLOSE_POSITION(pose, x, y, z) ( CLOSE(GetXPosition(pose), x) && CLOSE(GetYPosition(pose), y) && CLOSE(GetZPosition(pose), z))
#define CLOSE_ORIENTATION(pose, x, y, z, w) ( CLOSE(GetXOrientation(pose), x) && CLOSE(GetYOrientation(pose), y) && CLOSE(GetZOrientation(pose), z) && CLOSE(GetWOrientation(pose), w))

TEST_CASE( "Call to process (SVD Decomposition) Translation", "[processOnTranslation]" )
{
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 4;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
	cv::Mat projectionMatricesList[numberOfImages];
	projectionMatricesList[0] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[1] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 5,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[2] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 0,   0, 1, 0, 5,   0, 0, 1, 0);
	projectionMatricesList[3] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 5,   0, 1, 0, 5,   0, 0, 1, 0);
	cv::Mat homogeneousPoints3DMatrix = (cv::Mat_<float>(numberOfPoints, 4, CV_32FC1) << 0, 0, 5, 1,   50, 0, 5, 1,    0, 50, 5, 1,    50, 50, 5, 1);
	cv::Mat homogeneousPoints2DMatricesList[numberOfImages];
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		homogeneousPoints2DMatricesList[imageIndex] = cameraMatrix * projectionMatricesList[imageIndex] * homogeneousPoints3DMatrix.t();
		}

	//Initialize Inputs
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = NewCorrespondenceMaps2DSequence(); 
	for(int firstImageIndex = 0; firstImageIndex < numberOfImages; firstImageIndex++)
		{
		for(int secondImageIndex = firstImageIndex+1; secondImageIndex < numberOfImages; secondImageIndex++)
			{
			CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
			for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
				Point2D sourcePoint, sinkPoint;
				sourcePoint.x = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sourcePoint.y = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sinkPoint.x = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);
				sinkPoint.y = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	

				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(sourcePoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sourcePoint.y, &mandatoryOutput), 0) &&
					CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0)
					, "The test points should be interger pixel coordinates"
					);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Setup DFN
	svd->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/SvdDecomposition_Conf1.yaml");
	svd->configure();

	// Send input data to DFN
	svd->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	svd->process();

	// Query output data from DFN
	const Poses3DSequence& output = svd->posesSequenceOutput();
	bool success = svd->successOutput();

	REQUIRE(success);
	
	bool position1IsCorrect = CLOSE_POSITION(GetPose(output, 0), 0, 0, 0); 
	bool position2IsCorrect = CLOSE_POSITION(GetPose(output, 1), 5, 0, 0); 
	bool position3IsCorrect = CLOSE_POSITION(GetPose(output, 2), 0, 5, 0); 
	bool position4IsCorrect = CLOSE_POSITION(GetPose(output, 3), 5, 5, 0);
	REQUIRE(position1IsCorrect);
	REQUIRE(position2IsCorrect);
	REQUIRE(position3IsCorrect);
	REQUIRE(position4IsCorrect);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), 0, 0, 0, 1); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), 0, 0, 0, 1);
	REQUIRE(orientation1IsCorrect);
	REQUIRE(orientation2IsCorrect);
	REQUIRE(orientation3IsCorrect);
	REQUIRE(orientation4IsCorrect);

	// Cleanup
	delete(svd);
}

TEST_CASE( "Call to process (SVD Decomposition) RotoTranslation", "[processOnRotoTranslation]" )
{
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 4;
	float baseline = 5;
	float smallTranslation = 5;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
	Eigen::Quaternion<float> eigenQuaternion(0, 1, 0, 0); // w, x, y, z order

	Eigen::Matrix3f rotation = eigenQuaternion.toRotationMatrix();
	Eigen::Vector3f leftCameraPosition(0, -baseline, 2*baseline);		
	Eigen::Vector3f rightCameraPosition(baseline, -baseline, 2*baseline);

	cv::Mat projectionMatricesList[numberOfImages];
	projectionMatricesList[0] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[1] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, baseline,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[2] = (cv::Mat_<float>(3, 4, CV_32FC1) << 
		rotation(0,0), rotation(0,1), rotation(0,2), leftCameraPosition(0), 
		rotation(1,0), rotation(1,1), rotation(1,2), leftCameraPosition(1), 
		rotation(2,0), rotation(2,1), rotation(2,2), leftCameraPosition(2)
		);
	projectionMatricesList[3] = (cv::Mat_<float>(3, 4, CV_32FC1) << 
		rotation(0,0), rotation(0,1), rotation(0,2), rightCameraPosition(0), 
		rotation(1,0), rotation(1,1), rotation(1,2), rightCameraPosition(1), 
		rotation(2,0), rotation(2,1), rotation(2,2), rightCameraPosition(2) 
		);
	cv::Mat homogeneousPoints3DMatrix = (cv::Mat_<float>(numberOfPoints, 4, CV_32FC1) << 0, 0, 5, 1,   50, 0, 5, 1,    0, 50, 5, 1,    50, 50, 5, 1);
	cv::Mat homogeneousPoints2DMatricesList[numberOfImages];
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		homogeneousPoints2DMatricesList[imageIndex] = cameraMatrix * projectionMatricesList[imageIndex] * homogeneousPoints3DMatrix.t();
		}

	//Initialize Inputs
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = NewCorrespondenceMaps2DSequence(); 
	for(int firstImageIndex = 0; firstImageIndex < numberOfImages; firstImageIndex++)
		{
		for(int secondImageIndex = firstImageIndex+1; secondImageIndex < numberOfImages; secondImageIndex++)
			{
			CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
			for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
				Point2D sourcePoint, sinkPoint;
				sourcePoint.x = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sourcePoint.y = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sinkPoint.x = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);
				sinkPoint.y = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);

				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				
				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(sourcePoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sourcePoint.y, &mandatoryOutput), 0) &&
					CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0)
					, "The test points should be interger pixel coordinates"
					);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Setup DFN
	svd->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/SvdDecomposition_Conf1.yaml");
	svd->configure();

	// Send input data to DFN
	svd->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	svd->process();

	// Query output data from DFN
	const Poses3DSequence& output = svd->posesSequenceOutput();
	bool success = svd->successOutput();

	REQUIRE(success);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), eigenQuaternion.x(), eigenQuaternion.y(), eigenQuaternion.z(), eigenQuaternion.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), eigenQuaternion.x(), eigenQuaternion.y(), eigenQuaternion.z(), eigenQuaternion.w()); 
	REQUIRE(orientation1IsCorrect);
	REQUIRE(orientation2IsCorrect);
	REQUIRE(orientation3IsCorrect);
	REQUIRE(orientation4IsCorrect);

	bool position1IsCorrect = CLOSE_POSITION(GetPose(output, 0), 0, 0, 0); 
	bool position2IsCorrect = CLOSE_POSITION(GetPose(output, 1), 5, 0, 0); 
	bool position3IsCorrect = CLOSE_POSITION(GetPose(output, 2), 0, -baseline, 2*baseline); 
	bool position4IsCorrect = CLOSE_POSITION(GetPose(output, 3), baseline, -baseline, 2*baseline);
	REQUIRE(position1IsCorrect);
	REQUIRE(position2IsCorrect);
	REQUIRE(position3IsCorrect);
	REQUIRE(position4IsCorrect);

	// Cleanup
	delete(svd);
}


TEST_CASE( "Call to process (SVD Decomposition) Rotation", "[processOnRotation]" )
{
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 4;
	float baseline = 5;
	float smallTranslation = 5;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
	Eigen::Quaternion<float> eigenQuaternion(0, 0, 0, 1); // w, x, y, z order

	Eigen::Matrix3f rotation = eigenQuaternion.toRotationMatrix();
	Eigen::Vector3f leftCameraPosition(0, 0, 0);		
	Eigen::Vector3f rightCameraPosition(-baseline, 0, 0);

	cv::Mat projectionMatricesList[numberOfImages];
	projectionMatricesList[0] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[1] = (cv::Mat_<float>(3, 4, CV_32FC1) << 1, 0, 0, baseline,   0, 1, 0, 0,   0, 0, 1, 0);
	projectionMatricesList[2] = (cv::Mat_<float>(3, 4, CV_32FC1) << 
		rotation(0,0), rotation(0,1), rotation(0,2), leftCameraPosition(0), 
		rotation(1,0), rotation(1,1), rotation(1,2), leftCameraPosition(1), 
		rotation(2,0), rotation(2,1), rotation(2,2), leftCameraPosition(2)
		);
	projectionMatricesList[3] = (cv::Mat_<float>(3, 4, CV_32FC1) << 
		rotation(0,0), rotation(0,1), rotation(0,2), rightCameraPosition(0), 
		rotation(1,0), rotation(1,1), rotation(1,2), rightCameraPosition(1), 
		rotation(2,0), rotation(2,1), rotation(2,2), rightCameraPosition(2) 
		);
	cv::Mat homogeneousPoints3DMatrix = (cv::Mat_<float>(numberOfPoints, 4, CV_32FC1) << 0, 0, 5, 1,   50, 0, 5, 1,    0, 50, 5, 1,    50, 50, 5, 1);
	cv::Mat homogeneousPoints2DMatricesList[numberOfImages];
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		homogeneousPoints2DMatricesList[imageIndex] = cameraMatrix * projectionMatricesList[imageIndex] * homogeneousPoints3DMatrix.t();
		}

	//Initialize Inputs
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = NewCorrespondenceMaps2DSequence(); 
	for(int firstImageIndex = 0; firstImageIndex < numberOfImages; firstImageIndex++)
		{
		for(int secondImageIndex = firstImageIndex+1; secondImageIndex < numberOfImages; secondImageIndex++)
			{
			CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
			for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
				Point2D sourcePoint, sinkPoint;
				sourcePoint.x = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sourcePoint.y = homogeneousPoints2DMatricesList[firstImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[firstImageIndex].at<float>(2, pointIndex);
				sinkPoint.x = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(0, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);
				sinkPoint.y = homogeneousPoints2DMatricesList[secondImageIndex].at<float>(1, pointIndex) / homogeneousPoints2DMatricesList[secondImageIndex].at<float>(2, pointIndex);

				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	
				
				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(sourcePoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sourcePoint.y, &mandatoryOutput), 0) &&
					CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0) && CLOSE(std::modf(sinkPoint.x, &mandatoryOutput), 0)
					, "The test points should be interger pixel coordinates"
					);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Setup DFN
	svd->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/SvdDecomposition_Conf1.yaml");
	svd->configure();

	// Send input data to DFN
	svd->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	svd->process();

	// Query output data from DFN
	const Poses3DSequence& output = svd->posesSequenceOutput();
	bool success = svd->successOutput();

	REQUIRE(success);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), eigenQuaternion.x(), eigenQuaternion.y(), eigenQuaternion.z(), eigenQuaternion.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), eigenQuaternion.x(), eigenQuaternion.y(), eigenQuaternion.z(), eigenQuaternion.w()); 
	REQUIRE(orientation1IsCorrect);
	REQUIRE(orientation2IsCorrect);
	REQUIRE(orientation3IsCorrect);
	REQUIRE(orientation4IsCorrect);

	bool position1IsCorrect = CLOSE_POSITION(GetPose(output, 0), 0, 0, 0); 
	bool position2IsCorrect = CLOSE_POSITION(GetPose(output, 1), 5, 0, 0); 
	bool position3IsCorrect = CLOSE_POSITION(GetPose(output, 2), 0, 0, 0); 
	bool position4IsCorrect = CLOSE_POSITION(GetPose(output, 3), -baseline, 0, 0);
	REQUIRE(position1IsCorrect);
	REQUIRE(position2IsCorrect);
	REQUIRE(position3IsCorrect);
	REQUIRE(position4IsCorrect);

	// Cleanup
	delete(svd);
}

/** @} */
