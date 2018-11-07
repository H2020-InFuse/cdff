/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN Registration3D/IcpMatcher
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <Registration3D/IcpMatcher.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

#include <boost/make_shared.hpp>

using namespace CDFF::DFN::Registration3D;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

#define REQUIRE_CLOSE(a,b) REQUIRE(a < b + 0.001); REQUIRE(a > b - 0.001);

TEST_CASE( "Call to process (Registration 3D Icp Matcher)", "[process]" )
{
	PRINT_TO_LOG("Running Icp matcher", "");

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
	IcpMatcher *icp = new IcpMatcher;

	// Setup DFN
	icp->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Registration3D/IcpMatcher_Conf1.yaml");
	icp->configure();

	// Send input data to DFN
	icp->sourceCloudInput(*sourcePointCloud);
	icp->sinkCloudInput(*sinkPointCloud);

	PRINT_TO_LOG("Running Icp matcher", "");
	// Run DFN
	icp->process();

	// Query output data from DFN
	const Pose3D& transform = icp->transformOutput();
	bool success = icp->successOutput();

	REQUIRE_CLOSE( GetXPosition(transform), 0);
	REQUIRE_CLOSE( GetYPosition(transform), 0);
	REQUIRE_CLOSE( GetZPosition(transform), 0);
	REQUIRE_CLOSE( GetXOrientation(transform), 0);
	REQUIRE_CLOSE( GetYOrientation(transform), 0);
	REQUIRE_CLOSE( GetZOrientation(transform), 0);
	REQUIRE_CLOSE( GetWOrientation(transform), 1);

	// Cleanup
	delete icp;
	delete sourcePointCloud;
	delete sinkPointCloud;
}

TEST_CASE( "Call to configure (Registration 3D Icp Matcher)", "[configure]" )
{
	// Instantiate DFN
	IcpMatcher *icp = new IcpMatcher;

	// Setup DFN
	icp->setConfigurationFile("../tests/ConfigurationFiles/DFNs/Registration3D/IcpMatcher_Conf1.yaml");
	icp->configure();

	// Cleanup
	delete icp;
}

/** @} */

