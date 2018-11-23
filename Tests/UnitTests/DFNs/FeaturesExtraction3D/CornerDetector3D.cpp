/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <FeaturesExtraction3D/CornerDetector3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

TEST_CASE( "DFN processing step succeeds (3D Corner detector)", "[process]" )
{
	// Prepare input data (a sphere)
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for (float alpha = 0; alpha < 2 * M_PI; alpha += 0.1)
	{
		for (float beta = 0; beta < 2 * M_PI; beta += 0.1)
		{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			inputCloud->points.push_back(spherePoint);
		}
	}
	*/

	// Prepare simpler input data (nothing)
	PointCloudConstPtr pc = NewPointCloud();

	// Instantiate DFN
	CornerDetector3D* harris = new CornerDetector3D;

	// Send input data to DFN
	harris->pointcloudInput(*pc);

	// Run DFN
	harris->process();

	// Query output data from DFN
	const VisualPointFeatureVector3D& output = harris->featuresOutput();

	// Cleanup
	delete harris;
	delete pc;
}

TEST_CASE( "DFN configuration succeeds (3D Corner detector)", "[configure]" )
{
	// Instantiate DFN
	CornerDetector3D* harris = new CornerDetector3D;

	// Setup DFN
	harris->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction3D/CornerDetector3D_Conf1.yaml");
	harris->configure();

	// Cleanup
	delete harris;
}

/** @} */
