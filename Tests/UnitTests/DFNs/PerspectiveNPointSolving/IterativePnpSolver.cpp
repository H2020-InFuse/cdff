/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN IterativePnpSolver
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PerspectiveNPointSolving/IterativePnpSolver.hpp>
#include <PointCloud.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/core/core.hpp>

using namespace dfn_ci;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Converters;

TEST_CASE( "Call to process (iterative PnP solver)", "[process]" )
{
	// Prepare input data (points in the 3D scene and their projections on the 2D image plane)
	const unsigned NUMBER_OF_POINTS = 8;
	PointCloud *points = new PointCloud;
	cv::Mat projections(NUMBER_OF_POINTS, 2, CV_32FC1);

	ClearPoints(*points);
	for (unsigned pointIndex = 0; pointIndex < NUMBER_OF_POINTS; pointIndex++)
	{
		AddPoint(*points, pointIndex * 0.1 + 0.1, pointIndex * 0.1, 3);
		projections.at<float>(pointIndex, 0) = pointIndex * 0.1;
		projections.at<float>(pointIndex, 1) = pointIndex * 0.1;
	}

	const VisualPointFeatureVector2D *projections_ =
		MatToVisualPointFeatureVector2DConverter().Convert(projections);

	// FIXME: Expected output
	Pose3D expectedCamera;
	SetPosition(expectedCamera, 0, 1, 2);
	SetOrientation(expectedCamera, 0, 0, 0, 1);

	// Instantiate DFN
	IterativePnpSolver *pnp = new IterativePnpSolver;

	// Send input data to DFN
	pnp->pointsInput(*points);
	pnp->projectionsInput(*projections_);

	// Run DFN
	pnp->process();

	// Query output data from DFN
	const Pose3D& camera = pnp->cameraOutput();
	bool success = pnp->successOutput();

	// FIXME: Tests
	REQUIRE( success == true );
	/*
	REQUIRE( GetXTranslation(camera) == GetXTranslation(expectedCamera) );
	REQUIRE( GetYTranslation(camera) == GetYTranslation(expectedCamera) );
	REQUIRE( GetZTranslation(camera) == GetZTranslation(expectedCamera) );
	REQUIRE( GetXOrientation(camera) == GetXOrientation(expectedCamera) );
	REQUIRE( GetYOrientation(camera) == GetYOrientation(expectedCamera) );
	REQUIRE( GetZOrientation(camera) == GetZOrientation(expectedCamera) );
	REQUIRE( GetWOrientation(camera) == GetWOrientation(expectedCamera) );
	*/

	// Cleanup
	delete pnp;
	delete points;
	delete projections_;
}

TEST_CASE( "Call to configure (iterative PnP solver)", "[configure]" )
{
	// Instantiate DFN
	IterativePnpSolver *pnp = new IterativePnpSolver;

	// Setup DFN
	pnp->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PerspectiveNPointSolving/IterativePnpSolver_Conf1.yaml");
	pnp->configure();

	// Cleanup
	delete pnp;
}

/** @} */
