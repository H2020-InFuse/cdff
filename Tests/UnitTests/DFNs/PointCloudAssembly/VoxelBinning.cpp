/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudAssembly/VoxelBinning.hpp>

using namespace CDFF::DFN::PointCloudAssembly;
using namespace PointCloudWrapper;


namespace VoxelBinningTest
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

using namespace VoxelBinningTest;

TEST_CASE( "DFN processing step succeeds no incremental mode(VoxelBinning)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 0.009, 0, 0);
	AddPoint(*secondCloud, 1.11, 1, 1);

	AddPoint(*firstCloud, -1, -1, -1);
	AddPoint(*secondCloud, -1, -1, -1);

	AddPoint(*firstCloud, 2, 3, 4);
	AddPoint(*firstCloud, 2.003, 3.006, 4.003);
	AddPoint(*secondCloud, 2, 3, 4);

	// Instantiate DFN, no incremental mode is default
	VoxelBinning* average = new VoxelBinning;

	// Send input data to DFN
	average->firstPointCloudInput(*firstCloud);
	average->secondPointCloudInput(*secondCloud);

	// Run DFN
	average->process();

	// Query output data from DFN
	const PointCloud& output = average->assembledCloudOutput();

	REQUIRE( GetNumberOfPoints(output) == 5);
	RequireExist( output, 0.0045, 0, 0);
	RequireExist( output, 1, 1, 1);
	RequireExist( output, -1, -1, -1);
	RequireExist( output, 1.11, 1, 1);
	RequireExist( output, 2.001, 3.002, 4.001);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with incremental mode (VoxelBinning)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 0.009, 0, 0);
	AddPoint(*secondCloud, 1.11, 1, 1);

	AddPoint(*firstCloud, -1, -1, -1);
	AddPoint(*secondCloud, -1, -1, -1);

	AddPoint(*firstCloud, 2, 3, 4);
	AddPoint(*firstCloud, 2.003, 3.006, 4.003);
	AddPoint(*secondCloud, 2, 3, 4);

	// Instantiate DFN
	VoxelBinning* average = new VoxelBinning;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/VoxelBinning_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 4);
	RequireExist( output, 0, 0, 0);
	RequireExist( output, -1, -1, -1);
	RequireExist( output, 1, 1, 1);
	RequireExist( output, 2.0015, 3.003, 4.0015);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output2) == 5);
	RequireExist( output2, 0.0045, 0, 0);
	RequireExist( output2, 1, 1, 1);
	RequireExist( output2, -1, -1, -1);
	RequireExist( output2, 1.11, 1, 1);
	RequireExist( output2, 2.00075, 3.0015, 4.00075);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}


TEST_CASE( "DFN processing step succeeds with distance filter (VoxelBinning)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	AddPoint(*firstCloud, 0, 0, 0);
	AddPoint(*firstCloud, 1, 1, 1);
	AddPoint(*secondCloud, 0.009, 0, 0);
	AddPoint(*secondCloud, 1.11, 1, 1);

	AddPoint(*firstCloud, -1, -1, -1);
	AddPoint(*secondCloud, -1, -1, -1);

	AddPoint(*firstCloud, 2, 3, 4);
	AddPoint(*firstCloud, 2.003, 3.006, 4.003);
	AddPoint(*secondCloud, 2, 3, 4);

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 1, 1, 1);
	float viewRadius1 = 1.7322;
	float viewRadius2 = 1.7299;

	// Instantiate DFN
	VoxelBinning* average = new VoxelBinning;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/VoxelBinning_Conf1.yaml");
	average->configure();

	// Adding First Input
	average->firstPointCloudInput(*firstCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius1);
	average->process();
	const PointCloud& output = average->assembledCloudOutput();

	//Testing first output
	REQUIRE( GetNumberOfPoints(output) == 2);
	RequireExist( output, 0, 0, 0);
	RequireExist( output, 1, 1, 1);

	// Adding Second Input
	average->firstPointCloudInput(*secondCloud);
	average->viewCenterInput(viewCenter);
	average->viewRadiusInput(viewRadius2);
	average->process();
	const PointCloud& output2 = average->assembledCloudOutput();

	// Testing second output
	REQUIRE( GetNumberOfPoints(output2) == 3);
	RequireExist( output2, 0.0045, 0, 0);
	RequireExist( output2, 1, 1, 1);
	RequireExist( output2, 1.11, 1, 1);

	// Cleanup
	delete average;
	delete firstCloud;
	delete secondCloud;
}

TEST_CASE( "DFN processing step succeeds empty (VoxelBinning)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr firstCloud = NewPointCloud();
	PointCloudPtr secondCloud = NewPointCloud();

	PoseWrapper::Pose3D viewCenter;
	PoseWrapper::SetPosition(viewCenter, 1, 1, 1);
	float viewRadius1 = 1.7322;
	float viewRadius2 = 1.7299;

	// Instantiate DFN
	VoxelBinning* average = new VoxelBinning;

	// Setup DFN, this sets incremental mode
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/VoxelBinning_Conf1.yaml");
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

TEST_CASE( "DFN configuration succeeds (VoxelBinning)", "[configure]" )
{
	// Instantiate DFN
	VoxelBinning* average = new VoxelBinning;

	// Setup DFN
	average->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudAssembly/VoxelBinning_Conf1.yaml");
	average->configure();

	// Cleanup
	delete average;
}

/** @} */
