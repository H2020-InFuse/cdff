/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN Registration3D/IcpCC
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <Registration3D/IcpCC.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

#include <boost/make_shared.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

TEST_CASE( "Call to process (Registration 3D Icp CC)", "[process]" )
{
	// Prepare input data (a sphere)
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
	PclPointCloudToPointCloudConverter pclPointCloudToPointCloud;
	PointCloudConstPtr sourcePointCloud = pclPointCloudToPointCloud.Convert(inputCloud);
	PointCloudConstPtr sinkPointCloud = pclPointCloudToPointCloud.Convert(inputCloud);

	// Instantiate DFN
	IcpCC *icp = new IcpCC;

	// Send input data to DFN
	icp->sourceCloudInput(*sourcePointCloud);
	icp->sinkCloudInput(*sinkPointCloud);

	// Run DFN
	icp->process();

	// Query output data from DFN
	const Pose3D& transform = icp->transformOutput();
	bool success = icp->successOutput();

	// Cleanup
	delete icp;
	delete sourcePointCloud;
	delete sinkPointCloud;
}

TEST_CASE( "Call to configure (Registration 3D Icp CC)", "[configure]" )
{
	// Instantiate DFN
	IcpCC *icp = new IcpCC;

	// Setup DFN
	icp->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Registration3D/IcpCC_Conf1.yaml");
	icp->configure();

	// Cleanup
	delete icp;
}

/** @} */
