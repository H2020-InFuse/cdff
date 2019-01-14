/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <FeaturesDescription3D/PfhDescriptor3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

using namespace CDFF::DFN::FeaturesDescription3D;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

TEST_CASE( "DFN processing step succeeds (3D PFH descriptor)", "[process]" )
{
	// Prepare input data (a sphere)
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for (float alpha = 0; alpha < 2 * M_PI; alpha += 0.01)
	{
		for (float beta = 0; beta < 2 * M_PI; beta += 0.01)
		{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			inputCloud->points.push_back(spherePoint);
		}
	}
	*/

	// Prepare input data (surface normals)
	/*
	pcl::PointCloud<pcl::Normal>::Ptr inputNormals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	*/

	// Prepare simpler input data (pointcloud and surface normals)
	PointCloudPtr pc = new PointCloud;
	ClearPoints(*pc);
	AddPoint(*pc, 0, 0, 0);

	PointCloudPtr normals = new PointCloud;
	ClearPoints(*normals);

	// Prepare input data (keypoints)
	VisualPointFeatureVector3DPtr features = new VisualPointFeatureVector3D;
	ClearPoints(*features);
	AddPoint(*features, 0);

	// Instantiate DFN
	PfhDescriptor3D *pfh = new PfhDescriptor3D;

	// Send input data to DFN
	pfh->pointcloudInput(*pc);
	pfh->normalsInput(*normals);
	pfh->featuresInput(*features);

	// Run DFN
	pfh->process();

	// Query output data from DFN
	const VisualPointFeatureVector3D& output = pfh->featuresOutput();

	REQUIRE( GetNumberOfPoints(output) == 1 );

	// Cleanup
	delete pfh;
	delete pc;
	delete normals;
	delete features;
}

TEST_CASE( "DFN configuration succeeds (3D PFH descriptor)", "[configure]" )
{
	// Instantiate DFN
	PfhDescriptor3D *pfh = new PfhDescriptor3D;

	// Setup DFN
	pfh->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/PfhDescriptor3D_Conf1.yaml");
	pfh->configure();

	// Cleanup
	delete pfh;
}

TEST_CASE( "DFN configuration fails because of conflicting parameters: computation of surface normals is disabled and forced at the same time", "[configure][bad config]" )
{
	// Instantiate DFN
	PfhDescriptor3D *pfh = new PfhDescriptor3D;

	// Setup DFN
	pfh->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/PfhDescriptor3D_ConflictingGeneralParameters.yaml");
	REQUIRE_THROWS( pfh->configure() );

	// Cleanup
	delete pfh;
}

TEST_CASE( "DFN configuration fails because of conflicting normal estimation parameters", "[configure][bad config]" )
{
	// Instantiate DFN
	PfhDescriptor3D *pfh = new PfhDescriptor3D;

	// Setup DFN
	pfh->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/PfhDescriptor3D_ConflictingNormalEstimationParameters.yaml");
	REQUIRE_THROWS( pfh->configure() );

	// Cleanup
	delete pfh;
}

TEST_CASE( "DFN processing fails on account of the provided normals being inadequate and normal estimation being disabled", "[process][bad input]")
{
	// Instantiate DFN
	PfhDescriptor3D *pfh = new PfhDescriptor3D;

	// Setup DFN
	pfh->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/PfhDescriptor3D_DisabledNormalEstimation.yaml");
	pfh->configure();

	// Prepare input data
	PointCloudPtr pc = new PointCloud;
	ClearPoints(*pc);
	AddPoint(*pc, 0, 0, 0);

	PointCloudPtr normals = new PointCloud;
	ClearPoints(*normals);

	VisualPointFeatureVector3DPtr features = new VisualPointFeatureVector3D;
	ClearPoints(*features);
	AddPoint(*features, 0);

	// Send input data to DFN
	pfh->pointcloudInput(*pc);
	pfh->featuresInput(*features);
	pfh->normalsInput(*normals);

	// Run DFN
	REQUIRE_THROWS( pfh->process() );

	// Cleanup
	delete pfh;
	delete pc;
	delete features;
	delete normals;
}

/** @} */
