/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN Triangulation.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Catch/catch.h>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Errors/Assert.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>
#include <DataGenerators/SyntheticGenerators/CameraPair.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace DataGenerators;
/* --------------------------------------------------------------------------
 *
 * Test Class
 *
 * --------------------------------------------------------------------------
 */
class TriangulationTest 
	{
	public:
		static void RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences = 20);

	private:
		static const double EPSILON;
		static Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache;
		static Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter;

		static void SetupMocksAndStubs();
		static void ValidateOutput(PointCloudConstPtr output, cv::Mat pointCloud);
	};

void TriangulationTest::RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences)
	{
	SetupMocksAndStubs();

	CameraPair cameraPair;
	cameraPair.SetSecondCameraPose(secondCameraPose);

	cv::Mat cvFundamentalMatrix = cameraPair.GetFundamentalMatrix();
	cv::Mat pointCloud;
	CorrespondenceMap2DConstPtr input = cameraPair.GetSomeRandomCorrespondences(numberOfCorrespondences, pointCloud);

	Triangulation triangulation;
	triangulation.correspondenceMapInput(input);
	triangulation.poseInput(secondCameraPose);
	triangulation.process();

	PointCloudConstPtr output = triangulation.pointCloudOutput();
	ValidateOutput(output, pointCloud);
	
	delete(input);
	delete(output);
	}

const double TriangulationTest::EPSILON = 0.0001;
Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* TriangulationTest::stubTriangulationPoseCache = NULL;
Mocks::Pose3DToMatConverter* TriangulationTest::mockTriangulationPoseConverter = NULL;

void TriangulationTest::SetupMocksAndStubs()
	{
	static bool setupDone = false;
	if (setupDone)
		{
		return;
		}
	setupDone = true;

	stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);
	}

#define REQUIRE_CLOSE(a, b) \
	REQUIRE( a <= b + EPSILON); \
	REQUIRE( a >= b - EPSILON); \

void TriangulationTest::ValidateOutput(PointCloudConstPtr output, cv::Mat pointCloud)
	{
	REQUIRE ( GetNumberOfPoints(*output) == pointCloud.cols );

	for(unsigned pointIndex = 0; pointIndex < pointCloud.cols; pointIndex++)
		{
		REQUIRE_CLOSE( pointCloud.at<float>(0, pointIndex), GetXCoordinate(*output, pointIndex) );
		REQUIRE_CLOSE( pointCloud.at<float>(1, pointIndex), GetYCoordinate(*output, pointIndex) );
		REQUIRE_CLOSE( pointCloud.at<float>(2, pointIndex), GetZCoordinate(*output, pointIndex) );
		}
	}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process for translation", "[processTranslation]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 0, 1);

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);

	delete(secondCameraPose);
	}

TEST_CASE( "Success Call to process for rototranslation", "[processRototranslation]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 1, 0);

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);

	delete(secondCameraPose);
	}

TEST_CASE( "Success Call to process for rototranslation 2", "[processRototranslation2]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 1, 0);
	SetOrientation(*secondCameraPose, 0, 0, std::sin(M_PI/4), std::sin(M_PI/4));

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);

	delete(secondCameraPose);
	}

/** @} */
