/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN Triangulation
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <DataGenerators/SyntheticGenerators/CameraPair.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::PointCloudReconstruction2DTo3D;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace DataGenerators;

class TriangulationTest
{
	public:
		static void RandomCorrespondencesTest(const Pose3D& secondCameraPose, unsigned numberOfCorrespondences = 20);

	private:
		static void ValidateOutput(const PointCloud& output, cv::Mat pointCloud);
};

void TriangulationTest::RandomCorrespondencesTest(const Pose3D& secondCameraPose, unsigned numberOfCorrespondences)
{
	// Prepare input data
	CameraPair cameraPair;
	cameraPair.SetSecondCameraPose(&secondCameraPose);

	cv::Mat cvFundamentalMatrix = cameraPair.GetFundamentalMatrix();
	cv::Mat pointCloud;
	const CorrespondenceMap2D *input = cameraPair.GetSomeRandomCorrespondences(numberOfCorrespondences, pointCloud);

	// Instantiate DFN
	Triangulation *triangulation = new Triangulation;;

	// Send input data to DFN
	triangulation->matchesInput(*input);
	triangulation->poseInput(secondCameraPose);

	// Run DFN
	triangulation->process();

	// Query output data from DFN
	const PointCloud& output = triangulation->pointcloudOutput();

	// Tests
	ValidateOutput(output, pointCloud);

	// Cleanup
	delete triangulation;
	delete input;
}

void TriangulationTest::ValidateOutput(const PointCloud& output, cv::Mat pointCloud)
{
	REQUIRE ( GetNumberOfPoints(output) == pointCloud.cols );

	for (unsigned pointIndex = 0; pointIndex < pointCloud.cols; pointIndex++)
	{
		REQUIRE( GetXCoordinate(output, pointIndex) == Approx(pointCloud.at<float>(0, pointIndex)) );
		REQUIRE( GetYCoordinate(output, pointIndex) == Approx(pointCloud.at<float>(1, pointIndex)) );
		REQUIRE( GetZCoordinate(output, pointIndex) == Approx(pointCloud.at<float>(2, pointIndex)) );
	}
}

TEST_CASE( "Success Call to process for translation", "[processTranslation]" )
{
	Pose3D secondCameraPose;
	SetPosition(secondCameraPose, 1, 0, 0);
	SetOrientation(secondCameraPose, 0, 0, 0, 1);

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);
}

TEST_CASE( "Success Call to process for rototranslation", "[processRototranslation]" )
{
	Pose3D secondCameraPose;
	SetPosition(secondCameraPose, 1, 0, 0);
	SetOrientation(secondCameraPose, 0, 0, 1, 0);

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);
}

TEST_CASE( "Success Call to process for rototranslation 2", "[processRototranslation2]" )
{
	Pose3D secondCameraPose;
	SetPosition(secondCameraPose, 1, 1, 0);
	SetOrientation(secondCameraPose, 0, 0, std::sin(M_PI/4), std::sin(M_PI/4));

	TriangulationTest::RandomCorrespondencesTest(secondCameraPose, 20);
}

/** @} */
