/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CeresAdjustment.cpp
 * @date 06/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN implementation CeresAdjustment.
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
#include <BundleAdjustment/CeresAdjustment.hpp>
#include <MatToFrameConverter.hpp>
#include <Eigen/Dense>

using namespace dfn_ci;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Ceres Adjustment) and fail", "[failprocess]" )
{
	//Initialize Inputs
	CorrespondenceMaps2DSequencePtr correspondenceMapsSequence = NewCorrespondenceMaps2DSequence(); 
	for(int index = 0; index < 6; index++)
		{
		CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
		AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
		}

	// Instantiate DFN
	CeresAdjustment* ceres = new CeresAdjustment;

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

	REQUIRE(!success);

	// Cleanup
	delete(ceres);
}

TEST_CASE( "Call to configure (Ceres Adjustment)", "[configure]" )
{
	// Instantiate DFN
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf1.yaml");
	ceres->configure();

	// Cleanup
	delete(ceres);
}

#define EPSILON 0.001
#define GROUND_TRUTH_EPSILON 0.01
#define CLOSE_GROUND_TRUTH(a, b) (b > a - GROUND_TRUTH_EPSILON && b < a + GROUND_TRUTH_EPSILON)
#define CLOSE(a, b) (b > a - EPSILON && b < a + EPSILON)
#define CLOSE_POSITION(pose, x, y, z) ( CLOSE(GetXPosition(pose), x) && CLOSE(GetYPosition(pose), y) && CLOSE(GetZPosition(pose), z))
#define CLOSE_HALF_ORIENTATION(pose, x, y, z, w) ( CLOSE(GetXOrientation(pose), x) && CLOSE(GetYOrientation(pose), y) && CLOSE(GetZOrientation(pose), z) && CLOSE(GetWOrientation(pose), w))
#define CLOSE_ORIENTATION(pose, x, y, z, w) ( CLOSE_HALF_ORIENTATION(pose, x, y, z, w) || CLOSE_HALF_ORIENTATION(pose, -x, -y, -z, -w) )

TEST_CASE( "Call to process (Ceres Adjustment) Translation", "[processOnTranslation]" )
{
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 5;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraPosition(0, baseline, 0);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraOrientation = 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f rightCameraPosition = secondCameraPosition + secondCameraOrientation.inverse() * baselineVector;
	Eigen::Vector3f pointsVector[5] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5), Eigen::Vector3f(10, 10, 2.5) };
	Eigen::Vector3f transformedPointVector[4][5];
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - secondCameraPosition) );
		transformedPointVector[3][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - rightCameraPosition) );
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
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf1.yaml");
	ceres->configure();

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

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
	delete(ceres);
}

TEST_CASE( "Call to process (Ceres Adjustment) RotoTranslation", "[processOnRotoTranslation]" )
{	
	//This test does not pass without initialization, Ceres fails to converge
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 5;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 10, 0, 0,   0, 10, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraPosition(0, -baseline, 3*baseline);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraOrientation = 
		Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f rightCameraPosition = secondCameraPosition + secondCameraOrientation.inverse() * baselineVector;
	Eigen::Vector3f pointsVector[5] = {Eigen::Vector3f(5, 0, 10), Eigen::Vector3f(45, 450, 10), Eigen::Vector3f(20, 200, 10), Eigen::Vector3f(20, 150, 10), Eigen::Vector3f(20, 20, 5) };
	Eigen::Vector3f transformedPointVector[4][5];
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - secondCameraPosition) );
		transformedPointVector[3][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - rightCameraPosition) );
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
				
				// This is jus trying to extract the integer part as pixel coordinates. I cannot use the normal cast because I got something like static_cast<int>(-10) = -9.
				double decimalPart[4] = { sourcePoint.x, sourcePoint.y, sinkPoint.x, sinkPoint.y };
				double integerPart[4] = {0, 0, 0, 0};
				for(int index = 0; index < 4; index++)
					{
					while (decimalPart[index] > 0.5 || decimalPart[index] <= -0.5)
						{
						integerPart[index] += (decimalPart[index] > 0.5 ? 1 : -1);
						decimalPart[index] += (decimalPart[index] > 0.5 ? -1 : 1);					
						}
					ASSERT( CLOSE_GROUND_TRUTH(decimalPart[index], 0), "The test points should be interger pixel coordinates");
					}
				sourcePoint.x = integerPart[0];
				sourcePoint.y = integerPart[1];
				sinkPoint.x = integerPart[2];
				sinkPoint.y = integerPart[3];
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	//Preparing initial pose estimates
	Poses3DSequencePtr initialPosesSequence = NewPoses3DSequence();
	Pose3DPtr secondCameraPose = NewPose3D();
	SetPosition(*secondCameraPose, 0.01, -4.99, 14.98);
	SetOrientation(*secondCameraPose, 0.99999, 0.00001, 0, 0);
	AddPose(*initialPosesSequence, *secondCameraPose);

	//Preparing initial points estimates;
	PointCloudPtr initialPointCloud = NewPointCloud();
	AddPoint(*initialPointCloud, 5.02, 0, 10);
	AddPoint(*initialPointCloud, 4.98, 450, 10);
	AddPoint(*initialPointCloud, 20.03, 200, 10);
	AddPoint(*initialPointCloud, 19.99, 150, 10);
	AddPoint(*initialPointCloud, 20, 20, 5.01);	

	// Instantiate DFN
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf2.yaml");
	ceres->configure();

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);
	ceres->guessedPosesSequenceInput(*initialPosesSequence);
	ceres->guessedPointCloudInput(*initialPointCloud);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

	REQUIRE(success);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	REQUIRE(orientation1IsCorrect);
	REQUIRE(orientation2IsCorrect);
	REQUIRE(orientation3IsCorrect);
	REQUIRE(orientation4IsCorrect);

	bool position1IsCorrect = CLOSE_POSITION(GetPose(output, 0), 0, 0, 0); 
	bool position2IsCorrect = CLOSE_POSITION(GetPose(output, 1), 5, 0, 0); 
	bool position3IsCorrect = CLOSE_POSITION(GetPose(output, 2), 0, -baseline, 3*baseline); 
	bool position4IsCorrect = CLOSE_POSITION(GetPose(output, 3), baseline, -baseline, 3*baseline);
	REQUIRE(position1IsCorrect);
	REQUIRE(position2IsCorrect);
	REQUIRE(position3IsCorrect);
	REQUIRE(position4IsCorrect);

	// Cleanup
	delete(ceres);
}


TEST_CASE( "Call to process (Ceres Adjustment) Rotation", "[processOnRotation]" )
{	
	// This test does not pass, Ceres fails to converge
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 5;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraPosition(0, 0, 0);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraOrientation = 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f rightCameraPosition = secondCameraPosition + secondCameraOrientation.inverse() * baselineVector;
	Eigen::Vector3f pointsVector[5] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5), Eigen::Vector3f(20, 20, 2.5) };
	Eigen::Vector3f transformedPointVector[4][5];
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - secondCameraPosition) );
		transformedPointVector[3][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - rightCameraPosition) );
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

				// This is jus trying to extract the integer part as pixel coordinates. I cannot use the normal cast because I got something like static_cast<int>(-10) = -9.
				double decimalPart[4] = { sourcePoint.x, sourcePoint.y, sinkPoint.x, sinkPoint.y };
				double integerPart[4] = {0, 0, 0, 0};
				for(int index = 0; index < 4; index++)
					{
					while (decimalPart[index] > 0.5 || decimalPart[index] <= -0.5)
						{
						integerPart[index] += (decimalPart[index] > 0.5 ? 1 : -1);
						decimalPart[index] += (decimalPart[index] > 0.5 ? -1 : 1);					
						}
					ASSERT( CLOSE(decimalPart[index], 0), "The test points should be interger pixel coordinates");
					}
				sourcePoint.x = integerPart[0];
				sourcePoint.y = integerPart[1];
				sinkPoint.x = integerPart[2];
				sinkPoint.y = integerPart[3];
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	//Preparing initial pose estimates
	Poses3DSequencePtr initialPosesSequence = NewPoses3DSequence();
	Pose3DPtr secondCameraPose = NewPose3D();
	SetPosition(*secondCameraPose, 5, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 1, 0);
	AddPose(*initialPosesSequence, *secondCameraPose);

	//Preparing initial points estimates;
	PointCloudPtr initialPointCloud = NewPointCloud();
	AddPoint(*initialPointCloud, 0, 0, 5);
	AddPoint(*initialPointCloud, 50, 0, 5);
	AddPoint(*initialPointCloud, 0, 50, 5);
	AddPoint(*initialPointCloud, 50, 50, 5);
	AddPoint(*initialPointCloud, 20, 20, 2.5);

	// Instantiate DFN
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf1.yaml");
	ceres->configure();

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);
	ceres->guessedPosesSequenceInput(*initialPosesSequence);
	ceres->guessedPointCloudInput(*initialPointCloud);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

	REQUIRE(success);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
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
	delete(ceres);
}


TEST_CASE( "Call to process (Ceres Adjustment) 90 Degree Rotation", "[processOn90DegreeRotation]" )
{
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 5;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 10, 0, 0,   0, 10, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraPosition(-baseline, 0, baseline);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraOrientation = 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f rightCameraPosition = secondCameraPosition + secondCameraOrientation.inverse() * baselineVector;
	Eigen::Vector3f pointsVector[5] = {Eigen::Vector3f(5, 0, 10), Eigen::Vector3f(45, 450, 10), Eigen::Vector3f(20, 200, 10), Eigen::Vector3f(20, 150, 10), Eigen::Vector3f(20, 20, 12.5) };
	Eigen::Vector3f transformedPointVector[4][5];
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - secondCameraPosition) );
		transformedPointVector[3][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - rightCameraPosition) );
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
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf2.yaml");
	ceres->configure();

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

	REQUIRE(success);
	
	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	REQUIRE(orientation1IsCorrect);
	REQUIRE(orientation2IsCorrect);
	REQUIRE(orientation3IsCorrect);
	REQUIRE(orientation4IsCorrect);

	bool position1IsCorrect = CLOSE_POSITION(GetPose(output, 0), 0, 0, 0); 
	bool position2IsCorrect = CLOSE_POSITION(GetPose(output, 1), 5, 0, 0); 
	bool position3IsCorrect = CLOSE_POSITION(GetPose(output, 2), -baseline, 0, baseline); 
	bool position4IsCorrect = CLOSE_POSITION(GetPose(output, 3), -baseline, 0, 0);
	REQUIRE(position1IsCorrect);
	REQUIRE(position2IsCorrect);
	REQUIRE(position3IsCorrect);
	REQUIRE(position4IsCorrect);


	//TRY TO recompute the transform with the value of the computation
	// Cleanup
	delete(ceres);
}



TEST_CASE( "Call to process (Ceres Adjustment) Intruders", "[processOnIntruders]" )
{	
	// This test does not pass, Ceres fails to converge
	//Initialize Data
	int numberOfImages = 4;
	int numberOfPoints = 5;
	float baseline = 5;
	float smallTranslation = 5;
	Eigen::Matrix3f cameraMatrix;
	cameraMatrix << 1, 0, 0,   0, 1, 0,    0, 0, 1;
	Eigen::Vector3f secondCameraPosition(0, 0, 0);
	Eigen::Vector3f baselineVector(baseline, 0, 0);
	Eigen::Quaternion<float> secondCameraOrientation = 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f rightCameraPosition = secondCameraPosition + secondCameraOrientation.inverse() * baselineVector;
	Eigen::Vector3f pointsVector[5] = {Eigen::Vector3f(0, 0, 5), Eigen::Vector3f(50, 0, 5), Eigen::Vector3f(0, 50, 5), Eigen::Vector3f(50, 50, 5), Eigen::Vector3f(20, 20, 2.5) };
	Eigen::Vector3f transformedPointVector[4][5];
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		transformedPointVector[0][pointIndex] = cameraMatrix * pointsVector[pointIndex];
		transformedPointVector[1][pointIndex] = cameraMatrix * (pointsVector[pointIndex] - baselineVector);
		transformedPointVector[2][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - secondCameraPosition) );
		transformedPointVector[3][pointIndex] = cameraMatrix * (secondCameraOrientation * (pointsVector[pointIndex] - rightCameraPosition) );
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

				// This is jus trying to extract the integer part as pixel coordinates. I cannot use the normal cast because I got something like static_cast<int>(-10) = -9.
				double decimalPart[4] = { sourcePoint.x, sourcePoint.y, sinkPoint.x, sinkPoint.y };
				double integerPart[4] = {0, 0, 0, 0};
				for(int index = 0; index < 4; index++)
					{
					while (decimalPart[index] > 0.5 || decimalPart[index] <= -0.5)
						{
						integerPart[index] += (decimalPart[index] > 0.5 ? 1 : -1);
						decimalPart[index] += (decimalPart[index] > 0.5 ? -1 : 1);					
						}
					ASSERT( CLOSE(decimalPart[index], 0), "The test points should be interger pixel coordinates");
					}
				sourcePoint.x = integerPart[0];
				sourcePoint.y = integerPart[1];
				sinkPoint.x = integerPart[2];
				sinkPoint.y = integerPart[3];
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);

				//This is an intruder point, it represents a point appearing only in the first correspondence, it is supposed to be discarded
				if (firstImageIndex == 0 && secondImageIndex == 1 && pointIndex == 2)
					{
					Point2D sourceIntruderPoint, sinkIntruderPoint;
					sourceIntruderPoint.x = 58;
					sourceIntruderPoint.y = 58;
					sinkIntruderPoint.x = 109;
					sinkIntruderPoint.x = 109;
					AddCorrespondence(*correspondenceMap, sourceIntruderPoint, sinkIntruderPoint, 1);
					}
				}
			AddCorrespondenceMap(*correspondenceMapsSequence, *correspondenceMap);
			}
		}

	//Preparing initial pose estimates
	Poses3DSequencePtr initialPosesSequence = NewPoses3DSequence();
	Pose3DPtr secondCameraPose = NewPose3D();
	SetPosition(*secondCameraPose, 5, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 1, 0);
	AddPose(*initialPosesSequence, *secondCameraPose);

	//Preparing initial points estimates;
	PointCloudPtr initialPointCloud = NewPointCloud();
	AddPoint(*initialPointCloud, 0, 0, 5);
	AddPoint(*initialPointCloud, 50, 0, 5);
	AddPoint(*initialPointCloud, 454.1, 50.99, 5.88); //Intruder point
	AddPoint(*initialPointCloud, 0, 50, 5);
	AddPoint(*initialPointCloud, 50, 50, 5);
	AddPoint(*initialPointCloud, 20, 20, 2.5);

	// Instantiate DFN
	CeresAdjustment* ceres = new CeresAdjustment;

	// Setup DFN
	ceres->setConfigurationFile("../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_Conf1.yaml");
	ceres->configure();

	// Send input data to DFN
	ceres->correspondenceMapsSequenceInput(*correspondenceMapsSequence);
	ceres->guessedPosesSequenceInput(*initialPosesSequence);
	ceres->guessedPointCloudInput(*initialPointCloud);

	// Run DFN
	ceres->process();

	// Query output data from DFN
	const Poses3DSequence& output = ceres->posesSequenceOutput();
	bool success = ceres->successOutput();

	REQUIRE(success);

	bool orientation1IsCorrect = CLOSE_ORIENTATION(GetPose(output, 0), 0, 0, 0, 1); 
	bool orientation2IsCorrect = CLOSE_ORIENTATION(GetPose(output, 1), 0, 0, 0, 1); 
	bool orientation3IsCorrect = CLOSE_ORIENTATION(GetPose(output, 2), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
	bool orientation4IsCorrect = CLOSE_ORIENTATION(GetPose(output, 3), secondCameraOrientation.x(), secondCameraOrientation.y(), secondCameraOrientation.z(), secondCameraOrientation.w()); 
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
	delete(ceres);
}

/** @} */
