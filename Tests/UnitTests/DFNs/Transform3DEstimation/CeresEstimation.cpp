/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CeresEstimation.cpp
 * @date 24/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN CeresEstimation.
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
#include <Transform3DEstimation/CeresEstimation.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Types/CPP/CorrespondenceMaps3DSequence.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::Transform3DEstimation;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Class
 *
 * --------------------------------------------------------------------------
 */
class CeresEstimationTest
	{
	public:
		static void RandomCorrespondencesTest(cv::Mat transform, unsigned numberOfCorrespondences = 20);
		static void DoubleRandomCorrespondencesTest(cv::Mat transform1, cv::Mat transform2, unsigned numberOfCorrespondences = 20);
		static void TripleRandomCorrespondencesTest(cv::Mat transform1, cv::Mat transform2, cv::Mat transform3, unsigned numberOfCorrespondences = 20, float noise = 0);
		static void RandomCorrespondencesWithOneFailureTest(cv::Mat transform, unsigned numberOfCorrespondences = 20);
		static void NotEnoughPoointsFailureTest();
		static void BadPointsFailureTest();
		static void FixedPointTest();

	private:
		static const double EPSILON;
		static void ValidateOutput(const PoseWrapper::Pose3D& output, cv::Mat transform);
	};

void CeresEstimationTest::RandomCorrespondencesTest(cv::Mat transform, unsigned numberOfCorrespondences)
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
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

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);
	REQUIRE( GetNumberOfPoses(output) == 1);
	ValidateOutput(GetPose(output, 0), transform);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

void CeresEstimationTest::DoubleRandomCorrespondencesTest(cv::Mat transform1, cv::Mat transform2, unsigned numberOfCorrespondences)
	{
	CorrespondenceMap3DPtr input1 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input2 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input3 = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(3,0) = 1;

		cv::Mat transformedPoint3d1 = transform1*point3d;
		cv::Mat transformedPoint3d2 = transform2*point3d;
		Point3D source, sink, secondSink;
		source.x = point3d.at<float>(0,0);
		source.y = point3d.at<float>(1,0);
		source.z = point3d.at<float>(2,0);
		sink.x = transformedPoint3d1.at<float>(0,0);
		sink.y = transformedPoint3d1.at<float>(1,0);
		sink.z = transformedPoint3d1.at<float>(2,0);
		secondSink.x = transformedPoint3d2.at<float>(0,0);
		secondSink.y = transformedPoint3d2.at<float>(1,0);
		secondSink.z = transformedPoint3d2.at<float>(2,0);
		AddCorrespondence(*input1, source, sink, 1.0);
		AddCorrespondence(*input2, source, secondSink, 1.0);
		AddCorrespondence(*input3, sink, secondSink, 1.0);
		}

	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input1);
	AddCorrespondenceMap(*sequence, *input2);
	AddCorrespondenceMap(*sequence, *input3);

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);
	REQUIRE( GetNumberOfPoses(output) == 3);
	ValidateOutput(GetPose(output, 0), transform1);
	ValidateOutput(GetPose(output, 1), transform2);
	
	cv::Mat lastRow = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
	cv::Mat matrixList[2] = {transform1, lastRow};
	cv::vconcat(matrixList, 2, transform1);
	matrixList[0] = transform2;
	cv::vconcat(matrixList, 2, transform2);
	cv::Mat linkTransform = transform2 * transform1.inv();
	ValidateOutput(GetPose(output, 2), linkTransform(cv::Rect(0, 0, 4, 3)));

	delete(sequence);
	delete(input1);
	delete(input2);
	delete(input3);
	delete(squaresMinimization);
	}

void CeresEstimationTest::TripleRandomCorrespondencesTest(cv::Mat transform1, cv::Mat transform2, cv::Mat transform3, unsigned numberOfCorrespondences, float noise)
	{
	CorrespondenceMap3DPtr input1 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input2 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input3 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input4 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input5 = NewCorrespondenceMap3D();
	CorrespondenceMap3DPtr input6 = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(3,0) = 1;

		cv::Mat transformedPoint3d1 = transform1*point3d;
		cv::Mat transformedPoint3d2 = transform2*point3d;
		cv::Mat transformedPoint3d3 = transform3*point3d;
		Point3D source, sink, secondSink, thirdSink;
		source.x = point3d.at<float>(0,0) + ( (float)(rand()%10) - 5) * noise;
		source.y = point3d.at<float>(1,0) + ( (float)(rand()%10) - 5) * noise;
		source.z = point3d.at<float>(2,0) + ( (float)(rand()%10) - 5) * noise;
		sink.x = transformedPoint3d1.at<float>(0,0) + ( (float)(rand()%10) - 5) * noise;
		sink.y = transformedPoint3d1.at<float>(1,0) + ( (float)(rand()%10) - 5) * noise;
		sink.z = transformedPoint3d1.at<float>(2,0) + ( (float)(rand()%10) - 5) * noise;
		secondSink.x = transformedPoint3d2.at<float>(0,0) + ( (float)(rand()%10) - 5) * noise;
		secondSink.y = transformedPoint3d2.at<float>(1,0) + ( (float)(rand()%10) - 5) * noise;
		secondSink.z = transformedPoint3d2.at<float>(2,0) + ( (float)(rand()%10) - 5) * noise;
		thirdSink.x = transformedPoint3d3.at<float>(0,0) + ( (float)(rand()%10) - 5) * noise;
		thirdSink.y = transformedPoint3d3.at<float>(1,0) + ( (float)(rand()%10) - 5) * noise;
		thirdSink.z = transformedPoint3d3.at<float>(2,0) + ( (float)(rand()%10) - 5) * noise;
		AddCorrespondence(*input1, source, sink, 1.0);
		AddCorrespondence(*input2, source, secondSink, 1.0);
		AddCorrespondence(*input3, source, thirdSink, 1.0);
		AddCorrespondence(*input4, sink, secondSink, 1.0);
		AddCorrespondence(*input5, sink, thirdSink, 1.0);
		AddCorrespondence(*input6, secondSink, thirdSink, 1.0);
		}

	CorrespondenceMaps3DSequencePtr sequence = NewCorrespondenceMaps3DSequence();
	AddCorrespondenceMap(*sequence, *input1);
	AddCorrespondenceMap(*sequence, *input2);
	AddCorrespondenceMap(*sequence, *input3);
	AddCorrespondenceMap(*sequence, *input4);
	AddCorrespondenceMap(*sequence, *input5);
	AddCorrespondenceMap(*sequence, *input6);

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);
	REQUIRE( GetNumberOfPoses(output) == 6);
	ValidateOutput(GetPose(output, 0), transform1);
	ValidateOutput(GetPose(output, 1), transform2);
	ValidateOutput(GetPose(output, 2), transform3);
	
	cv::Mat lastRow = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
	cv::Mat matrixList[2] = {transform1, lastRow};
	cv::vconcat(matrixList, 2, transform1);
	matrixList[0] = transform2;
	cv::vconcat(matrixList, 2, transform2);
	matrixList[0] = transform3;
	cv::vconcat(matrixList, 2, transform3);

	cv::Mat linkTransform12 = transform2 * transform1.inv();
	cv::Mat linkTransform13 = transform3 * transform1.inv();
	cv::Mat linkTransform23 = transform3 * transform2.inv();
	ValidateOutput(GetPose(output, 3), linkTransform12(cv::Rect(0, 0, 4, 3)));
	ValidateOutput(GetPose(output, 4), linkTransform13(cv::Rect(0, 0, 4, 3)));
	ValidateOutput(GetPose(output, 5), linkTransform23(cv::Rect(0, 0, 4, 3)));

	delete(sequence);
	delete(input1);
	delete(input2);
	delete(input3);
	delete(input4);
	delete(input5);
	delete(input6);
	delete(squaresMinimization);
	}

void CeresEstimationTest::RandomCorrespondencesWithOneFailureTest(cv::Mat transform, unsigned numberOfCorrespondences)
	{
	CorrespondenceMap3DPtr input = NewCorrespondenceMap3D();
	for(int index = 0; index < numberOfCorrespondences; index++)
		{
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(1,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
		point3d.at<float>(2,0) = ( (float)(rand()%10) + 0.1) * 0.1 + 2;
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
	AddCorrespondenceMap(*sequence, *input2);

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == false);

	delete(sequence);
	delete(input);
	delete(input2);
	delete(squaresMinimization);
	}

void CeresEstimationTest::NotEnoughPoointsFailureTest()
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

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == false);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

void CeresEstimationTest::BadPointsFailureTest()
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

	CeresEstimation* squaresMinimization = new CeresEstimation();
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

void CeresEstimationTest::FixedPointTest()
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

	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Transform3DEstimation/CeresEstimation_Conf2.yaml");
	squaresMinimization->configure();
	squaresMinimization->matchesInput(*sequence);
	squaresMinimization->process();

	const Poses3DSequence& output = squaresMinimization->transformsOutput();
	bool success = squaresMinimization->successOutput();
	REQUIRE(success == true);

	delete(sequence);
	delete(input);
	delete(squaresMinimization);
	}

const double CeresEstimationTest::EPSILON = 0.01;

#define REQUIRE_CLOSE(a, b) \
	REQUIRE( a <= b + EPSILON); \
	REQUIRE( a >= b - EPSILON); \

void CeresEstimationTest::ValidateOutput(const PoseWrapper::Pose3D& output, cv::Mat transform)
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
TEST_CASE( "Success Call to process for Translation (Ceres Estimation)", "[processTranslation]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 1, 0, -5,
		0, 0, 1, 2 );

	CeresEstimationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for Rototranslation (Ceres Estimation)", "[processRototranslation]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 0, -1, -5,
		0, 1, 0, 2 );

	CeresEstimationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for RotoTranslation 2 (Ceres Estimation)", "[processRototranslation2]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	CeresEstimationTest::RandomCorrespondencesTest(transform);
	}

TEST_CASE( "Success Call to process for Double RotoTranslation 2 (Ceres Estimation)", "[processDoubleRototranslation2]" )
	{
	cv::Mat transform1 = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	cv::Mat transform2 = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 0, -1, -5,
		0, 1, 0, 2 );

	CeresEstimationTest::DoubleRandomCorrespondencesTest(transform1, transform2);
	}

TEST_CASE( "Success Call to process for Triple RotoTranslation 2 (Ceres Estimation)", "[processTripleRototranslation2]" )
	{
	cv::Mat transform1 = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	cv::Mat transform2 = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 0, -1, -5,
		0, 1, 0, 2 );

	cv::Mat transform3 = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 1, 0, -5,
		0, 0, 1, 2 );

	CeresEstimationTest::TripleRandomCorrespondencesTest(transform1, transform2, transform3);
	}

TEST_CASE( "Success Call to process for Triple RotoTranslation 2 With Noise(Ceres Estimation)", "[processTripleRototranslation2Noise]" )
	{
	cv::Mat transform1 = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	cv::Mat transform2 = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 0, -1, -5,
		0, 1, 0, 2 );

	cv::Mat transform3 = (cv::Mat_<float>(3,4) <<
		1, 0, 0, 5,
		0, 1, 0, -5,
		0, 0, 1, 2 );

	CeresEstimationTest::TripleRandomCorrespondencesTest(transform1, transform2, transform3, 20, 0.00001);
	}

TEST_CASE( "Success Call to process for RotoTranslation 2 with failure (Ceres Estimation)", "[processRototranslation2]" )
	{
	cv::Mat transform = (cv::Mat_<float>(3,4) <<
		0, 0, -1, 5,
		0, 1, 0, -5,
		-1, 0, 0, 2 );

	CeresEstimationTest::RandomCorrespondencesWithOneFailureTest(transform);
	}

TEST_CASE( "Success Call to process with fixed points (eres Estimation)", "[fixedPoint]" )
	{
	CeresEstimationTest::FixedPointTest();
	}

TEST_CASE( "Fail Call to process not enough points (Ceres Estimation)", "[processFail]" )
	{
	CeresEstimationTest::NotEnoughPoointsFailureTest();
	}

TEST_CASE( "Fail Call to process bad points (Ceres Estimation)", "[processFail]" )
	{
	CeresEstimationTest::BadPointsFailureTest();
	}

TEST_CASE( "Call to configure (Ceres Estimation)", "[configure]" )
	{
	CeresEstimation* squaresMinimization = new CeresEstimation();
	squaresMinimization->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Transform3DEstimation/CeresEstimation_Conf1.yaml");
	squaresMinimization->configure();
	delete(squaresMinimization);
	}

/** @} */
