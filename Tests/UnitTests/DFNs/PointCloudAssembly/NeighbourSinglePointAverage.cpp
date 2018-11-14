/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudAssembly/NeighbourSinglePointAverage.hpp>

using namespace CDFF::DFN::PointCloudAssembly;
using namespace PointCloudWrapper;


namespace NeighbourSinglePointAverageTest
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

using namespace NeighbourSinglePointAverageTest;

TEST_CASE( "DFN processing step succeeds no incremental mode(NeighbourSinglePointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, -1, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 0.1, 0, 0);
	AddPoint(*firstCloud, 0.2, 0, 0);

	AddPoint(*secondCloud, 0, 0.006, 0);
	AddPoint(*secondCloud, 0.1, 0.006, 0);
	AddPoint(*secondCloud, 0.101, 0.006, 0);
	AddPoint(*secondCloud, 0.2, 0.006, 0);
	AddPoint(*secondCloud, 1, 0.006, 0);

	// Instantiate DFN, no incremental mode is default
	NeighbourSinglePointAverage* average = new NeighbourSinglePointAverage;

	// Send input data to DFN
	average->firstPointCloudInput(*firstCloud);
	average->secondPointCloudInput(*secondCloud);

	// Run DFN
	average->process();

	// Query output data from DFN
	const PointCloud& output = average->assembledCloudOutput();

	REQUIRE( GetNumberOfPoints(output) == 7);
	RequireExist( output, 0, 0.003, 0);
	RequireExist( output, 0.1, 0.003, 0);
	RequireExist( output, 0.2, 0.003, 0);
	RequireExist( output, -0.001, 0.003, 0);
	RequireExist( output, -1, 0.003, 0);
	RequireExist( output, 0.101, 0.003, 0);
	RequireExist( output, 1, 0.003, 0);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with incremental mode (NeighbourSinglePointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, -1, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 0.1, 0, 0);
	AddPoint(*firstCloud, 0.2, 0, 0);

	AddPoint(*secondCloud, 0, 0.006, 0);
	AddPoint(*secondCloud, 0.1, 0.006, 0);
	AddPoint(*secondCloud, 0.101, 0.006, 0);
	AddPoint(*secondCloud, 0.2, 0.006, 0);
	AddPoint(*secondCloud, 1, 0.006, 0);

	// Instantiate DFN
	NeighbourSinglePointAverage* average = new NeighbourSinglePointAverage;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourSinglePointAverage_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 5);
	RequireExist( output, -1, 0, 0);
	RequireExist( output, -0.001, 0, 0);
	RequireExist( output, 0, 0, 0);
	RequireExist( output, 0.1, 0, 0);
	RequireExist( output, 0.2, 0, 0);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output) == 7);
	RequireExist( output, 0, 0.003, 0);
	RequireExist( output, 0.1, 0.003, 0);
	RequireExist( output, 0.2, 0.003, 0);
	RequireExist( output, -0.001, 0.003, 0);
	RequireExist( output, -1, 0.003, 0);
	RequireExist( output, 0.101, 0.003, 0);
	RequireExist( output, 1, 0.003, 0);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with distance filter (NeighbourSinglePointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, -1, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 0.1, 0, 0);
	AddPoint(*firstCloud, 0.2, 0, 0);

	AddPoint(*secondCloud, 0, 0.006, 0);
	AddPoint(*secondCloud, 0.1, 0.006, 0);
	AddPoint(*secondCloud, 0.101, 0.006, 0);
	AddPoint(*secondCloud, 0.2, 0.006, 0);
	AddPoint(*secondCloud, 1, 0.006, 0);

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 0.1, 0, 0);
	float viewRadius1 = 0.1001;
	float viewRadius2 = 0.25;

	// Instantiate DFN
	NeighbourSinglePointAverage* average = new NeighbourSinglePointAverage;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourSinglePointAverage_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius1);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 3);
	RequireExist( output, 0, 0, 0);
	RequireExist( output, 0.1, 0, 0);
	RequireExist( output, 0.2, 0, 0);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius2);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output) == 5);
	RequireExist( output, 0, 0.003, 0);
	RequireExist( output, 0.1, 0.003, 0);
	RequireExist( output, 0.2, 0.003, 0);
	RequireExist( output, -0.001, 0.003, 0);
	RequireExist( output, 0.101, 0.003, 0);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}

TEST_CASE( "DFN processing step succeeds empty (NeighbourSinglePointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 1, 1, 1);
	float viewRadius1 = 1.7322;
	float viewRadius2 = 1.7299;

	// Instantiate DFN
	NeighbourSinglePointAverage* average = new NeighbourSinglePointAverage;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourSinglePointAverage_Conf1.yaml");
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

TEST_CASE( "DFN configuration succeeds (NeighbourSinglePointAverage)", "[configure]" )
{
	// Instantiate DFN
	NeighbourSinglePointAverage* average = new NeighbourSinglePointAverage;

	// Setup DFN
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourSinglePointAverage_Conf1.yaml");
	average->configure();

	// Cleanup
	delete average;
}

/** @} */
