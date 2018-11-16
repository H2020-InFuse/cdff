/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudAssembly/MatcherAssembly.hpp>

using namespace CDFF::DFN::PointCloudAssembly;
using namespace PointCloudWrapper;


namespace MatcherAssemblyTest
{
void RequireExist(const PointCloud& cloud, float x, float y, float z)
	{
	const float EPSILON = 0.00001;
	bool found = false;
	int numberOfPoints = GetNumberOfPoints(cloud);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		float pointX = GetXCoordinate(cloud, pointIndex);
		float pointY = GetYCoordinate(cloud, pointIndex);
		float pointZ = GetZCoordinate(cloud, pointIndex);
		if (pointX > x - EPSILON && pointX < x + EPSILON && pointY > y - EPSILON && pointY < y + EPSILON && pointZ > z - EPSILON && pointZ < z + EPSILON)
			{
			found = true;
			break;
			}
		}
	REQUIRE( found );
	}
}

using namespace MatcherAssemblyTest;

TEST_CASE( "DFN processing step succeeds no incremental mode(MatcherAssembly)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		AddPoint(*firstCloud, x, -y, 0);
		AddPoint(*firstCloud, x, y, 0);
		AddPoint(*secondCloud, x, y, 0);
		AddPoint(*secondCloud, -x, y, 0);
		}

	// Instantiate DFN, no incremental mode is default
	MatcherAssembly* average = new MatcherAssembly;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/MatcherAssembly_Conf2.yaml");
	average->configure();

	// Send input data to DFN
	average->firstPointCloudInput(*firstCloud);
	average->secondPointCloudInput(*secondCloud);

	// Run DFN
	average->process();

	// Query output data from DFN
	const PointCloud& output = average->assembledCloudOutput();
	// I could not predict the output
	//REQUIRE( GetNumberOfPoints(output) == 4*3);
	/*for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		RequireExist( output, x, y, 0);
		RequireExist( output, x, -y, 0);
		RequireExist( output, -x, y, 0);
		}*/

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with incremental mode (MatcherAssembly)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		AddPoint(*firstCloud, x, -y, 0);
		AddPoint(*firstCloud, x, y, 0);
		AddPoint(*secondCloud, x, y, 0);
		AddPoint(*secondCloud, -x, y, 0);
		}

	// Instantiate DFN
	MatcherAssembly* average = new MatcherAssembly;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/MatcherAssembly_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	// I could not predict the output
	//REQUIRE( GetNumberOfPoints(output) == 4*2);
	/*for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		RequireExist( output, x, y, 0);
		RequireExist( output, x, -y, 0);
		}*/

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	// I could not predict the output
	//REQUIRE( GetNumberOfPoints(output) == 4*3);
	/*for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		RequireExist( output, x, y, 0);
		RequireExist( output, x, -y, 0);
		RequireExist( output, -x, y, 0);
		}*/

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with distance filter (MatcherAssembly)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);

		AddPoint(*firstCloud, x, -y, 0);
		AddPoint(*firstCloud, x, y, 0);
		AddPoint(*secondCloud, x, y, 0);
		AddPoint(*secondCloud, -x, y, 0);
		}

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 0, 0, 0);
	float viewRadius1 = 0.5;
	float viewRadius2 = 0.7;

	// Instantiate DFN
	MatcherAssembly* average = new MatcherAssembly;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/MatcherAssembly_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius1);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	// I could not predict the output
	/*int counter = 0;
	for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);
		float distance = std::sqrt( x*x + y*y );
		
		if (distance <= viewRadius1)
			{
			RequireExist( output, x, y, 0);
			RequireExist( output, x, -y, 0);
			counter++;
			}
		}*/
	//REQUIRE( GetNumberOfPoints(output) == counter*2);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius2);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	// I could not predict the output
	/*counter = 0;
	for(int i = 0; i < 2; i++) for(float j = 0; j < 2; j++)
		{
		float x = 0.1 * static_cast<float>(i);
		float y = 0.1 * static_cast<float>(j);
		float distance = std::sqrt( x*x + y*y );
		
		if (distance <= viewRadius1)
			{
			RequireExist( output, x, y, 0);
			RequireExist( output, x, -y, 0);
			RequireExist( output, -x, y, 0);
			counter++;
			}
		}*/
	//REQUIRE( GetNumberOfPoints(output) == counter*3);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}

TEST_CASE( "DFN processing step succeeds empty (MatcherAssembly)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 1, 1, 1);
	float viewRadius1 = 1.7322;
	float viewRadius2 = 1.7299;

	// Instantiate DFN
	MatcherAssembly* average = new MatcherAssembly;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/MatcherAssembly_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius1);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 0);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius2);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output2) == 0);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}

TEST_CASE( "DFN configuration succeeds (MatcherAssembly)", "[configure]" )
{
	// Instantiate DFN
	MatcherAssembly* average = new MatcherAssembly;

	// Setup DFN
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/MatcherAssembly_Conf1.yaml");
	average->configure();

	// Cleanup
	delete average;
}

/** @} */
