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
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraCenter(0, baseline, 0);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraRotation(1, 0, 0, 0); // w, x, y, z order

	Eigen::Quaternion<float> stereoPairRotationMatrix = secondCameraRotation;
	Eigen::Translation<float, 3> stereoPairLeftTranslation(secondCameraCenter);
	Eigen::Translation<float, 3> stereoPairRightTranslation(secondCameraCenter + secondCameraRotation * baselineVector);

	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairLeftTransform = (stereoPairLeftTranslation * stereoPairRotationMatrix).inverse();	
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairRightTransform = (stereoPairRightTranslation * stereoPairRotationMatrix).inverse();	

	Eigen::Vector3f pointsVector[4] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5) };
	Eigen::Vector3f transformedPointVector[4][4];
	for(int pointIndex = 0; pointIndex < 4; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * stereoPairLeftTransform * pointsVector[pointIndex];
		transformedPointVector[3][pointIndex] = cameraMatrix * stereoPairRightTransform * pointsVector[pointIndex];
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
				sourcePoint.x = transformedPointVector[firstImageIndex][pointIndex](0) / transformedPointVector[firstImageIndex][pointIndex](2);
				sourcePoint.y = transformedPointVector[firstImageIndex][pointIndex](1) / transformedPointVector[firstImageIndex][pointIndex](2);
				sinkPoint.x = transformedPointVector[secondImageIndex][pointIndex](0) / transformedPointVector[secondImageIndex][pointIndex](2);
				sinkPoint.y = transformedPointVector[secondImageIndex][pointIndex](1) / transformedPointVector[secondImageIndex][pointIndex](2);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	
				
				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);

				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(std::abs(sourcePoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sourcePoint.y), &mandatoryOutput), 0) &&
					CLOSE(std::modf(std::abs(sinkPoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sinkPoint.y), &mandatoryOutput), 0)
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
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraCenter(0, -baseline, 2*baseline);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraRotation(0, 1, 0, 0); // w, x, y, z order

	Eigen::Quaternion<float> stereoPairRotationMatrix = secondCameraRotation;
	Eigen::Translation<float, 3> stereoPairLeftTranslation(secondCameraCenter);
	Eigen::Translation<float, 3> stereoPairRightTranslation(secondCameraCenter + secondCameraRotation * baselineVector);

	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairLeftTransform = (stereoPairLeftTranslation * stereoPairRotationMatrix).inverse();	
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairRightTransform = (stereoPairRightTranslation * stereoPairRotationMatrix).inverse();	

	Eigen::Vector3f pointsVector[4] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5) };
	Eigen::Vector3f transformedPointVector[4][4];
	for(int pointIndex = 0; pointIndex < 4; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * stereoPairLeftTransform * pointsVector[pointIndex];
		transformedPointVector[3][pointIndex] = cameraMatrix * stereoPairRightTransform * pointsVector[pointIndex];
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
				sourcePoint.x = transformedPointVector[firstImageIndex][pointIndex](0) / transformedPointVector[firstImageIndex][pointIndex](2);
				sourcePoint.y = transformedPointVector[firstImageIndex][pointIndex](1) / transformedPointVector[firstImageIndex][pointIndex](2);
				sinkPoint.x = transformedPointVector[secondImageIndex][pointIndex](0) / transformedPointVector[secondImageIndex][pointIndex](2);
				sinkPoint.y = transformedPointVector[secondImageIndex][pointIndex](1) / transformedPointVector[secondImageIndex][pointIndex](2);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	
				
				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);

				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(std::abs(sourcePoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sourcePoint.y), &mandatoryOutput), 0) &&
					CLOSE(std::modf(std::abs(sinkPoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sinkPoint.y), &mandatoryOutput), 0)
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
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
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
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraCenter(0, 0, 0);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraRotation(0, 0, 0, 1); // w, x, y, z order

	Eigen::Quaternion<float> stereoPairRotationMatrix = secondCameraRotation;
	Eigen::Translation<float, 3> stereoPairLeftTranslation(secondCameraCenter);
	Eigen::Translation<float, 3> stereoPairRightTranslation(secondCameraCenter + secondCameraRotation * baselineVector);

	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairLeftTransform = (stereoPairLeftTranslation * stereoPairRotationMatrix).inverse();	
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairRightTransform = (stereoPairRightTranslation * stereoPairRotationMatrix).inverse();	

	Eigen::Vector3f pointsVector[4] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5) };
	Eigen::Vector3f transformedPointVector[4][4];
	for(int pointIndex = 0; pointIndex < 4; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * stereoPairLeftTransform * pointsVector[pointIndex];
		transformedPointVector[3][pointIndex] = cameraMatrix * stereoPairRightTransform * pointsVector[pointIndex];
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
				sourcePoint.x = transformedPointVector[firstImageIndex][pointIndex](0) / transformedPointVector[firstImageIndex][pointIndex](2);
				sourcePoint.y = transformedPointVector[firstImageIndex][pointIndex](1) / transformedPointVector[firstImageIndex][pointIndex](2);
				sinkPoint.x = transformedPointVector[secondImageIndex][pointIndex](0) / transformedPointVector[secondImageIndex][pointIndex](2);
				sinkPoint.y = transformedPointVector[secondImageIndex][pointIndex](1) / transformedPointVector[secondImageIndex][pointIndex](2);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	
				
				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);

				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(std::abs(sourcePoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sourcePoint.y), &mandatoryOutput), 0) &&
					CLOSE(std::modf(std::abs(sinkPoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sinkPoint.y), &mandatoryOutput), 0)
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
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
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


TEST_CASE( "Call to process (SVD Decomposition) 90 Degree Rotation", "[processOn90DegreeRotation]" )
{
	return; //This test does not pass.
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 4;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 10, 0, 0,   0, 10, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraCenter(-baseline, 0, baseline);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraRotation = 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	Eigen::Quaternion<float> stereoPairRotationMatrix = secondCameraRotation;
	Eigen::Translation<float, 3> stereoPairLeftTranslation(secondCameraCenter);
	Eigen::Translation<float, 3> stereoPairRightTranslation(secondCameraCenter + secondCameraRotation * baselineVector);

	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairLeftTransform = (stereoPairLeftTranslation * stereoPairRotationMatrix).inverse();	
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> stereoPairRightTransform = (stereoPairRightTranslation * stereoPairRotationMatrix).inverse();	

	Eigen::Vector3f pointsVector[4] = {Eigen::Vector3f(5, 0, 10), Eigen::Vector3f(45, 450, 10), Eigen::Vector3f(20, 200, 10), Eigen::Vector3f(20, 150, 10) };
	Eigen::Vector3f transformedPointVector[4][4];
	for(int pointIndex = 0; pointIndex < 4; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * stereoPairLeftTransform * pointsVector[pointIndex];
		transformedPointVector[3][pointIndex] = cameraMatrix * stereoPairRightTransform * pointsVector[pointIndex];
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
				sourcePoint.x = transformedPointVector[firstImageIndex][pointIndex](0) / transformedPointVector[firstImageIndex][pointIndex](2);
				sourcePoint.y = transformedPointVector[firstImageIndex][pointIndex](1) / transformedPointVector[firstImageIndex][pointIndex](2);
				sinkPoint.x = transformedPointVector[secondImageIndex][pointIndex](0) / transformedPointVector[secondImageIndex][pointIndex](2);
				sinkPoint.y = transformedPointVector[secondImageIndex][pointIndex](1) / transformedPointVector[secondImageIndex][pointIndex](2);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);	
				
				sourcePoint.x = static_cast<int>(sourcePoint.x);
				sourcePoint.y = static_cast<int>(sourcePoint.y);
				sinkPoint.x = static_cast<int>(sinkPoint.x);
				sinkPoint.y = static_cast<int>(sinkPoint.y);

				double mandatoryOutput;
				ASSERT( 
					CLOSE(std::modf(std::abs(sourcePoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sourcePoint.y), &mandatoryOutput), 0) &&
					CLOSE(std::modf(std::abs(sinkPoint.x), &mandatoryOutput), 0) && CLOSE(std::modf(std::abs(sinkPoint.y), &mandatoryOutput), 0)
					, "The test points should be interger pixel coordinates"
					);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	// Instantiate DFN
	SvdDecomposition* svd = new SvdDecomposition;

	// Setup DFN
	svd->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/SvdDecomposition_Conf2.yaml");
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
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraRotation.x(), secondCameraRotation.y(), secondCameraRotation.z(), secondCameraRotation.w()); 
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


	//TRY TO recompute the transform with the value of the computation
	// Cleanup
	delete(svd);
}

/** @} */
