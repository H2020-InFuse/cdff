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
		static void FixedPointTest();

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

#define ADD_CORRESPONDENCE(map, x1, y1, z1, x2, y2, z2) \
	{ \
	Point3D source, sink; \
	source.x = x1; source.y = y1; source.z = z1; \
	sink.x = x2; sink.y = y2; sink.z = z2; \
	AddCorrespondence(*map, source, sink, 1); \
	}

void LeastSquaresMinimizationTest::FixedPointTest()
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	ADD_CORRESPONDENCE(input, 1.47883, -5.86605, 15.1224, 5.20526, -5.20025, 15.0052);
	ADD_CORRESPONDENCE(input, 1.20023, -5.82719, 15.1117, 4.89134, -5.14449, 14.9934);
	ADD_CORRESPONDENCE(input, 1.20023, -5.82719, 15.1117, 4.89134, -5.14449, 14.9934);
	ADD_CORRESPONDENCE(input, 1.21763, -5.77539, 15.1124, 4.92625, -5.11059, 14.9949);
	ADD_CORRESPONDENCE(input, -0.533516, 0.929886, 7.77014, 1.46368, 1.3499, 7.67232);
	ADD_CORRESPONDENCE(input, -0.578048, 0.920858, 7.76925, 1.41949, 1.33653, 7.67147);
	ADD_CORRESPONDENCE(input, -0.560237, 0.858443, 7.76961, 1.43716, 1.27052, 7.67181);
	ADD_CORRESPONDENCE(input, -3.44191, 5.5821, 15.4701, 0.200529, 6.36956, 15.615);
	ADD_CORRESPONDENCE(input, 1.48729, -5.77528, 15.3896, 5.18786, -4.99333, 15.0048);
	ADD_CORRESPONDENCE(input, -0.253229, 1.33958, 8.89021, 1.95397, 1.79218, 8.58852);
	ADD_CORRESPONDENCE(input, 0.791129, 0.933621, 7.7272, 2.79122, 1.36699, 7.50024);
	ADD_CORRESPONDENCE(input, -0.506791, 0.912102, 7.77067, 1.50348, 1.33535, 7.7412);
	ADD_CORRESPONDENCE(input, -0.578049, 0.911936, 7.76925, 1.41949, 1.32772, 7.67147);
	ADD_CORRESPONDENCE(input, -0.497882, 0.813961, 7.77084, 1.50348, 1.23756, 7.7412);
	ADD_CORRESPONDENCE(input, -0.506791, 0.903178, 7.77067, 1.50348, 1.3309, 7.7412);
	ADD_CORRESPONDENCE(input, 1.20167, -5.6012, 15.3161, 4.85985, -4.79907, 14.8764);
	ADD_CORRESPONDENCE(input, 1.09755, -4.42781, 15.3745, 4.76941, -3.66264, 14.9888);
	ADD_CORRESPONDENCE(input, 1.46141, -5.67477, 15.1218, 5.18781, -4.99325, 15.0046);
	ADD_CORRESPONDENCE(input, 1.47801, -4.51165, 15.6657, 5.2257, -3.73042, 15.2662);
	ADD_CORRESPONDENCE(input, -0.280949, 1.32591, 8.7995, 1.91521, 1.77942, 8.504);
	ADD_CORRESPONDENCE(input, -0.502299, 0.886328, 7.70179, 1.47716, 1.28577, 7.60568);
	ADD_CORRESPONDENCE(input, 1.44188, -4.53802, 15.6636, 5.11802, -3.68308, 15.002);
	ADD_CORRESPONDENCE(input, -0.502299, 0.77135, 7.70179, 1.47303, 1.18806, 7.53985);
	ADD_CORRESPONDENCE(input, -0.784654, 1.08013, 7.69624, 1.19692, 1.47687, 7.60027);
	ADD_CORRESPONDENCE(input, -1.12188, 2.20181, 8.43855, 1.00501, 2.6912, 8.48442);
	ADD_CORRESPONDENCE(input, -1.12188, 2.21633, 8.43852, 0.99538, 2.6242, 8.24335);
	ADD_CORRESPONDENCE(input, -1.2088, 2.056, 8.43668, 0.936669, 2.52512, 8.48294);

	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input);

	LeastSquaresMinimization* squaresMinimization = new LeastSquaresMinimization();
	squaresMinimization->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Transform3DEstimation/LeastSquaresMinimization_Conf2.yaml");
	squaresMinimization->configure();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);

	REQUIRE( GetNumberOfPoses(output) == 1 );
	Pose3D pose = GetPose(output, 0);
	PRINT_TO_LOG("pose", ToString(pose) );

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

TEST_CASE( "Success Call to process with fixed points (Least Squares Minimization)", "[fixedPoint]" )
	{
	LeastSquaresMinimizationTest::FixedPointTest();
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
