/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <FeaturesExtraction3D/IssDetector3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

TEST_CASE( "DFN processing step succeeds (3D ISS detector)", "[process]" )
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
	IssDetector3D* iss = new IssDetector3D;

	// Send input data to DFN
	iss->pointcloudInput(*pc);

	// Run DFN
	iss->process();

	// Query output data from DFN
	const VisualPointFeatureVector3D& output = iss->featuresOutput();

	// Cleanup
	delete iss;
	delete pc;
}

TEST_CASE( "DFN configuration succeeds (3D ISS detector)", "[configure]" )
{
	// Instantiate DFN
	IssDetector3D* iss = new IssDetector3D;

	// Setup DFN
	iss->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction3D/IssDetector3D_Conf1.yaml");
	iss->configure();

	// Cleanup
	delete iss;
}

/** @} */
