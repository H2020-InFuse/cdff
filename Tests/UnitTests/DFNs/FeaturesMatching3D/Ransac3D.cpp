/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN Ransac3D
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

#include <boost/make_shared.hpp>

using namespace CDFF::DFN::FeaturesMatching3D;
using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

TEST_CASE( "DFN processing step succeeds (3D RANSAC)", "[process]" )
{
	// Prepare input data
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for (unsigned pointIndex = 0; pointIndex < 10; pointIndex++)
	{
		pointCloud->points.push_back(pcl::PointXYZ(0.1*pointIndex, -0.1*pointIndex, 0));
		FeatureType feature;
		feature.histogram[0] = 0.001*pointIndex;
		feature.histogram[1] = 0.002*pointIndex;
		int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
		for (int componentIndex = 0; componentIndex < maxFeaturesNumber; componentIndex++)
		{
			feature.histogram[componentIndex] = 0;
		}
		featureCloud->points.push_back(feature);
	}

	PointCloudWithFeatures sourceCloud;
	PointCloudWithFeatures sinkCloud;
	sourceCloud.pointCloud = pointCloud;
	sinkCloud.pointCloud = pointCloud;
	sourceCloud.featureCloud = featureCloud;
	sinkCloud.featureCloud = featureCloud;
	sourceCloud.descriptorSize = 2;
	sinkCloud.descriptorSize = 2;
	*/

	// Prepare simpler input data
	VisualPointFeatureVector3DPtr sourceSet = new VisualPointFeatureVector3D;
	VisualPointFeatureVector3DPtr sinkSet = new VisualPointFeatureVector3D;

	// Instantiate DFN
	Ransac3D *ransac = new Ransac3D;

	// Send input data to DFN
	ransac->sourceFeaturesInput(*sourceSet);
	ransac->sinkFeaturesInput(*sinkSet);

	// Run DFN
	ransac->process();

	// Query output data from DFN
	const Pose3D& output = ransac->transformOutput();

	REQUIRE(GetXPosition(output) == 0);
	REQUIRE(GetYPosition(output) == 0);
	REQUIRE(GetZPosition(output) == 0);
	REQUIRE(GetXOrientation(output) == 0);
	REQUIRE(GetYOrientation(output) == 0);
	REQUIRE(GetZOrientation(output) == 0);
	REQUIRE(GetWOrientation(output) == 0);

	// Cleanup
	delete ransac;
	delete sourceSet;
	delete sinkSet;
}

TEST_CASE( "DFN configuration succeeds (3D RANSAC)", "[configure]" )
{
	// Instantiate DFN
	Ransac3D *ransac = new Ransac3D;

	// Setup DFN
	ransac->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/Ransac3D_Conf1.yaml");
	ransac->configure();
}

/** @} */
