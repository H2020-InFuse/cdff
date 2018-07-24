/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LeastSquaresMinimization.cpp
 * @date 24/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN LeastSquaresMinimization.
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
#include <Transform3DEstimation/LeastSquaresMinimization.hpp>
#include <PosesSequence.hpp>
#include <CorrespondenceMaps3DSequence.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Class
 *
 * --------------------------------------------------------------------------
 */
class LeastSquaresMinimizationTest
	{
	public:
		static void RandomCorrespondencesTest(cv::Mat transform, unsigned numberOfCorrespondences = 20);
		static void RandomCorrespondencesWithOneFailureTest(cv::Mat transform, unsigned numberOfCorrespondences = 20);
		static void NotEnoughPoointsFailureTest();
		static void BadPointsFailureTest();

	private:
		static const double EPSILON;
		static void ValidateOutput(const PoseWrapper::Pose3D& output, cv::Mat transform);
	};

void LeastSquaresMinimizationTest::RandomCorrespondencesTest(cv::Mat transform, unsigned numberOfCorrespondences)
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(3,0) = 1;

		cv::Mat transformedPoint3d = transform*point3d;
		Point3D source, sink;
		source.x = point3d.at<float>(0,0);
		source.y = point3d.at<float>(1,0);
		source.z = point3d.at<float>(2,0);
		sink.x = transformedPoint3d.at<float>(0,0);
		sink.y = transformedPoint3d.at<float>(1,0);
		sink.z = transformedPoint3d.at<float>(2,0);
		AddCorrespondence(*input, source, sink, 1.0);
		}

	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input);
	AddCorrespondenceMap(*sequence, *input);

	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);
	REQUIRE( GetNumberOfPoses(output) == 2);
	ValidateOutput(GetPose(output, 0), transform);
	ValidateOutput(GetPose(output, 1), transform);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

void LeastSquaresMinimizationTest::RandomCorrespondencesWithOneFailureTest(cv::Mat transform, unsigned numberOfCorrespondences)
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%100) + 1) * 0.1 + 2;
		point3d.at<float>(3,0) = 1;

		cv::Mat transformedPoint3d = transform*point3d;
		Point3D source, sink;
		source.x = point3d.at<float>(0,0);
		source.y = point3d.at<float>(1,0);
		source.z = point3d.at<float>(2,0);
		sink.x = transformedPoint3d.at<float>(0,0);
		sink.y = transformedPoint3d.at<float>(1,0);
		sink.z = transformedPoint3d.at<float>(2,0);
		AddCorrespondence(*input, source, sink, 1.0);
		}

	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input);

	CorrespondenceMap3DPtr input2 = NewCorrespondenceMap3D();
	for (unsigned index = 0; index < 20; index++)
		{
		Point3D source = { .x = 0 , .y= 0, .z = 0};
		Point3D sink = { .x = 0 , .y= 0, .z = 0};
		AddCorrespondence(*input, source, sink, 1);
		}
	AddCorrespondenceMap(*sequence, *input2);

	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);
	REQUIRE( GetNumberOfPoses(output) == 2);
	ValidateOutput(GetPose(output, 0), transform);

	REQUIRE( GetXPosition( GetPose(output, 1) ) == 0);
	REQUIRE( GetYPosition( GetPose(output, 1) ) == 0);
	REQUIRE( GetZPosition( GetPose(output, 1) ) == 0);
	REQUIRE( GetXOrientation( GetPose(output, 1) ) == 0);
	REQUIRE( GetYOrientation( GetPose(output, 1) ) == 0);
	REQUIRE( GetZOrientation( GetPose(output, 1) ) == 0);
	REQUIRE( GetWOrientation( GetPose(output, 1) ) == 0);

	delete(sequence);
	delete(input);
	delete(input2);
	delete(squaresMinimization);
	}

void LeastSquaresMinimizationTest::NotEnoughPoointsFailureTest()
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for (unsigned index = 0; index < 2; index++)
		{
		Point3D source = { .x = 0 , .y= 0, .z = 0};
		Point3D sink = { .x = 0 , .y= 0, .z = 0};
		AddCorrespondence(*input, source, sink, 1);
		}
	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input);

	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == false);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

void LeastSquaresMinimizationTest::BadPointsFailureTest()
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for (unsigned index = 0; index < 20; index++)
		{
		Point3D source = { .x = 0 , .y= 0, .z = 0};
		Point3D sink = { .x = 0 , .y= 0, .z = 0};
		AddCorrespondence(*input, source, sink, 1);
		}
	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input);

	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == false);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

const double LeastSquaresMinimizationTest::EPSILON = 0.0001;

#define REQUIRE_CLOSE(a, b) \
	REQUIRE( a <= b + EPSILON); \
	REQUIRE( a >= b - EPSILON); \

void LeastSquaresMinimizationTest::ValidateOutput(const PoseWrapper::Pose3D& output, cv::Mat transform)
	{
	cv::Mat rotationMatrix = transform( cv::Rect(0, 0, 3, 3) );
	cv::Mat translationMatrix = transform( cv::Rect(3, 0, 1, 3) );
	cv::Mat positionMatrix = - rotationMatrix.inv() * translationMatrix;
	float qw = std::sqrt(1.00 + rotationMatrix.at<float>(0,0) + rotationMatrix.at<float>(1,1) + rotationMatrix.at<float>(2,2)) / 2;
	float qx = (rotationMatrix.at<float>(2,1) - rotationMatrix.at<float>(1,2)) / ( 4 * qw );
	float qy = (rotationMatrix.at<float>(0,2) - rotationMatrix.at<float>(2,0)) / ( 4 * qw );
	float qz = (rotationMatrix.at<float>(1,0) - rotationMatrix.at<float>(0,1)) / ( 4 * qw );

	REQUIRE_CLOSE( GetXPosition(output), positionMatrix.at<float>(0,0) );
	REQUIRE_CLOSE( GetYPosition(output), positionMatrix.at<float>(1,0) );
	REQUIRE_CLOSE( GetZPosition(output), positionMatrix.at<float>(2,0) );
	REQUIRE_CLOSE( GetXOrientation(output), qx );
	REQUIRE_CLOSE( GetYOrientation(output), qy );
	REQUIRE_CLOSE( GetZOrientation(output), qz );
	REQUIRE_CLOSE( GetWOrientation(output), qw );
	}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process for Translation (Least Squares Minimization)", "[processTranslation]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 1, 0, -5,
		0, 0, 1, 2 );

	LeastSquaresMinimizationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for Rototranslation (Least Squares Minimization)", "[processRototranslation]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 0, -1, -5,
		0, 1, 0, 2 );

	LeastSquaresMinimizationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for RotoTranslation 2 (Least Squares Minimization)", "[processRototranslation2]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	LeastSquaresMinimizationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for RotoTranslation 2 with failure (Least Squares Minimization)", "[processRototranslation2]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	LeastSquaresMinimizationTest::RandomCorrespondencesWithOneFailureTest(transform);
	}

TEST_CASE( "Fail Call to process not enough points (Least Squares Minimization)", "[processFail]" )
	{
	LeastSquaresMinimizationTest::NotEnoughPoointsFailureTest();
	}

TEST_CASE( "Fail Call to process bad points (Least Squares Minimization)", "[processFail]" )
	{
	LeastSquaresMinimizationTest::BadPointsFailureTest();
	}

TEST_CASE( "Call to configure (Least Squares Minimization)", "[configure]" )
	{
	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Transform3DEstimation/LeastSquaresMinimization_Conf1.yaml");
	squaresMinimization->configure();
	delete(squaresMinimization);
	}

/** @} */
