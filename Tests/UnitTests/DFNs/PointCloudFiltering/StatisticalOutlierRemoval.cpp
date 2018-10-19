/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudFiltering/StatisticalOutlierRemoval.hpp>

using namespace CDFF::DFN::PointCloudFiltering;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace StatisticalOutlierRemovalTest
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
void RequireNotExist(const PointCloud& cloud, float x, float y, float z)
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
	REQUIRE( !found );
	}

float Next(float value, float limit, float resolution)
	{
	if (value + resolution < limit)
		{
		return value + resolution;
		}
	else
		{
		if (value >= limit)
			{
			return limit + resolution;
			}
		else
			{
			return limit;
			}
		}
	}
}

using namespace StatisticalOutlierRemovalTest;

TEST_CASE( "DFN simple processing step succeeds (StatisticalOutlierRemoval)", "[SimpleProcess]" )
{
	const float squareSize = 0.1;
	const float squareResolution = 0.001;
	int numberOfValidPoints = 0;
	//Input plane and noise
	PointCloudPtr cloud = NewPointCloud();
	for(float x = 0; x < squareSize; x = Next(x, squareSize, squareResolution))
		{
		for (float y = 0; y < squareSize; y = Next(y, squareSize, squareResolution))
			{
			AddPoint(*cloud, x, y, 0);
			numberOfValidPoints++;
			}
		}
	AddPoint(*cloud, 0, 0, 1);
	AddPoint(*cloud, 0, 1, -1);

	StatisticalOutlierRemoval* statisticalOutlierRemoval = new StatisticalOutlierRemoval;

	// Setup DFN
	statisticalOutlierRemoval->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudFiltering/StatisticalOutlierRemoval_Conf1.yaml");
	statisticalOutlierRemoval->configure();

	statisticalOutlierRemoval->pointCloudInput(*cloud);
	statisticalOutlierRemoval->process();
	const PointCloud& output = statisticalOutlierRemoval->filteredPointCloudOutput();

	REQUIRE( GetNumberOfPoints(output) == numberOfValidPoints);
	RequireNotExist( output, 0, 0, 1);
	RequireNotExist( output, 0, 1, -1);

	delete statisticalOutlierRemoval;
}

TEST_CASE( "DFN negative processing step succeeds (StatisticalOutlierRemoval)", "[process]" )
{
	const float squareSize = 0.1;
	const float squareResolution = 0.001;
	int numberOfValidPoints = 0;
	//Input plane and noise
	PointCloudPtr cloud = NewPointCloud();
	for(float x = 0; x < squareSize; x = Next(x, squareSize, squareResolution))
		{
		for (float y = 0; y < squareSize; y = Next(y, squareSize, squareResolution))
			{
			AddPoint(*cloud, x, y, 0);
			}
		numberOfValidPoints++;
		}
	AddPoint(*cloud, 0, 0, 1);
	AddPoint(*cloud, 0, 1, -1);

	StatisticalOutlierRemoval* statisticalOutlierRemoval = new StatisticalOutlierRemoval;

	// Setup DFN
	statisticalOutlierRemoval->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudFiltering/StatisticalOutlierRemoval_Conf2.yaml");
	statisticalOutlierRemoval->configure();

	statisticalOutlierRemoval->pointCloudInput(*cloud);
	statisticalOutlierRemoval->process();
	const PointCloud& output = statisticalOutlierRemoval->filteredPointCloudOutput();

	REQUIRE( GetNumberOfPoints(output) == 2);
	RequireExist( output, 0, 0, 1);
	RequireExist( output, 0, 1, -1);

	delete statisticalOutlierRemoval;
}

TEST_CASE( "DFN configuration succeeds (StatisticalOutlierRemoval)", "[configure]" )
{
	// Instantiate DFN
	StatisticalOutlierRemoval* statisticalOutlierRemoval = new StatisticalOutlierRemoval;

	// Setup DFN
	statisticalOutlierRemoval->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudFiltering/StatisticalOutlierRemoval_Conf1.yaml");
	statisticalOutlierRemoval->configure();

	// Cleanup
	delete statisticalOutlierRemoval;
}

/** @} */
