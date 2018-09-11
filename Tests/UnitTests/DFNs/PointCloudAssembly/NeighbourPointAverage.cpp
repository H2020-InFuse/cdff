/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudAssembly/NeighbourPointAverage.hpp>

using namespace CDFF::DFN::PointCloudAssembly;
using namespace PointCloudWrapper;


namespace NeighbourPointAverageTest
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

using namespace NeighbourPointAverageTest;

TEST_CASE( "DFN processing step succeeds no incremental mode(NeighbourPointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*secondCloud, 0.008, 0.002, 0.002);
	AddPoint(*secondCloud, 0.004, 0.004, 0.004);

	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 1.008, 1.002, 1.002);

	AddPoint(*firstCloud, 2, 2, 2);
	AddPoint(*secondCloud, 3, 3, 3);

	// Instantiate DFN, no incremental mode is default
	NeighbourPointAverage* average = new NeighbourPointAverage;

	// Send input data to DFN
	average->firstPointCloudInput(*firstCloud);
	average->secondPointCloudInput(*secondCloud);

	// Run DFN
	average->process();

	// Query output data from DFN
	const PointCloud& output = average->assembledCloudOutput();

	REQUIRE( GetNumberOfPoints(output) == 5);
	RequireExist( output, 0.002, 0.0015, 0.0015);
	RequireExist( output, 0.0030, 0.0015, 0.0015);
	RequireExist( output, 1.0040, 1.0010, 1.0010);
	RequireExist( output, 2.004, 2.001, 2.001);
	RequireExist( output, 2.996, 2.999, 2.999);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with incremental mode (NeighbourPointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*secondCloud, 0.008, 0.002, 0.002);
	AddPoint(*secondCloud, 0.004, 0.004, 0.004);

	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 1.008, 1.002, 1.002);

	AddPoint(*firstCloud, 2, 2, 2);
	AddPoint(*secondCloud, 3, 3, 3);

	// Instantiate DFN
	NeighbourPointAverage* average = new NeighbourPointAverage;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourPointAverage_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 4);
	RequireExist( output, 0, 0, 0);
	RequireExist( output, -0.001, 0, 0);
	RequireExist( output, 1, 1, 1);
	RequireExist( output, 2, 2, 2);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output2) == 5);
	RequireExist( output2, 0.002, 0.0015, 0.0015);
	RequireExist( output2, 0.0030, 0.0015, 0.0015);
	RequireExist( output2, 1.0040, 1.0010, 1.0010);
	RequireExist( output2, 2.004, 2.001, 2.001);
	RequireExist( output2, 2.996, 2.999, 2.999);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with distance filter (NeighbourPointAverage)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, -0.001, 0, 0);
	AddPoint(*secondCloud, 0.008, 0.002, 0.002);
	AddPoint(*secondCloud, 0.004, 0.004, 0.004);

	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 1.008, 1.002, 1.002);

	AddPoint(*firstCloud, 2, 2, 2);
	AddPoint(*secondCloud, 3, 3, 3);

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 1, 1, 1);
	float viewRadius1 = 1.7322;
	float viewRadius2 = 1.7299;

	// Instantiate DFN
	NeighbourPointAverage* average = new NeighbourPointAverage;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourPointAverage_Conf1.yaml");
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
	//RequireExist( output, -0.001, 0, 0);
	RequireExist( output, 1, 1, 1);
	RequireExist( output, 2, 2, 2);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius2);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output2) == 3);
	RequireExist( output2, 0.002, 0.0015, 0.0015);
	RequireExist( output2, 0.0030, 0.0015, 0.0015);
	RequireExist( output2, 1.0040, 1.0010, 1.0010);
	//RequireExist( output2, 2.004, 2.001, 2.001);
	//RequireExist( output2, 2.996, 2.999, 2.999);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}

TEST_CASE( "DFN configuration succeeds (NeighbourPointAverage)", "[configure]" )
{
	// Instantiate DFN
	NeighbourPointAverage* average = new NeighbourPointAverage;

	// Setup DFN
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/NeighbourPointAverage_Conf1.yaml");
	average->configure();

	// Cleanup
	delete average;
}

/** @} */
