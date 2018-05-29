/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN HarrisDetector3D.
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
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process (3D Harris detector)", "[process]" )
{
	// Prepare input data (a sphere)
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for (float alpha = 0; alpha < 2 * M_PI; alpha += 0.1)
	{
		for (float beta = 0; beta < 2*M_PI; beta += 0.1)
		{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			inputCloud->points.push_back(spherePoint);
		}
	}

	// Instantiate DFN
	HarrisDetector3D* harris = new HarrisDetector3D;

	// Send input data to DFN
	PointCloudConstPtr pc = new PointCloud;
	harris->pointcloudInput(*pc);

	// Run DFN
	harris->process();

	// Query output data from DFN
	const VisualPointFeatureVector3D& output = harris->featuresOutput();

	// Cleanup
	delete(harris);
	delete(pc);
}

TEST_CASE( "Call to configure (3D Harris detector)", "[configure]" )
{
	// Instantiate DFN
	HarrisDetector3D* harris = new HarrisDetector3D;

	// Setup DFN
	harris->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction3D/HarrisDetector3D_Conf1.yaml");
	harris->configure();

	// Cleanup
	delete(harris);
}

/** @} */
