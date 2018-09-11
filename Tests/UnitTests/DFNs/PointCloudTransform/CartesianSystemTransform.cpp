/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <PointCloudTransform/CartesianSystemTransform.hpp>

using namespace CDFF::DFN::PointCloudTransform;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CartesianSystemTransformTest
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

using namespace CartesianSystemTransformTest;

TEST_CASE( "DFN processing step succeeds (CartesianSystemTransform)", "[process]" )
{
	// Prepare simpler input data
	PointCloudPtr cloud = NewPointCloud();
	AddPoint(*cloud, 0, 0, 0);
	AddPoint(*cloud, 1, 0, 0);
	AddPoint(*cloud, 0, 1, 0);
	AddPoint(*cloud, 0, 0, 1);
	AddPoint(*cloud, 1, 1, 1);
	AddPoint(*cloud, 1, 2, 3);

	Pose3D pose;
	SetPosition(pose, 1, 2, 3);
	SetOrientation(pose, 0, 0, 0, 1);

	// Instantiate DFN, no incremental mode is default
	CartesianSystemTransform* cartesianTransform = new CartesianSystemTransform;

	// Running DFN
	cartesianTransform->pointCloudInput(*cloud);
	cartesianTransform->poseInput(pose);
	cartesianTransform->process();
	const PointCloud& output = cartesianTransform->transformedPointCloudOutput();

	//Testing Output
	REQUIRE( GetNumberOfPoints(output) == 6);
	RequireExist( output, 1, 2, 3);
	RequireExist( output, 2, 2, 3);
	RequireExist( output, 1, 3, 3);
	RequireExist( output, 1, 2, 4);
	RequireExist( output, 2, 3, 4);
	RequireExist( output, 2, 4, 6);

	//Changing Pose
	float rotationAngle = M_PI/3;
	float sinInvAngle = - std::sin(rotationAngle);
	float cosInvAngle = + std::cos(rotationAngle);
	SetPosition(pose, 0, 0, 0);
	SetOrientation(pose, std::sin(rotationAngle/2), 0, 0, std::cos(rotationAngle/2));
	
	// Running DFN
	cartesianTransform->pointCloudInput(*cloud);
	cartesianTransform->poseInput(pose);
	cartesianTransform->process();
	const PointCloud& output2 = cartesianTransform->transformedPointCloudOutput();	

	//Testing Output
	REQUIRE( GetNumberOfPoints(output2) == 6);
	RequireExist( output2, 0, 0*cosInvAngle - 0*sinInvAngle, 0*sinInvAngle + 0*cosInvAngle);
	RequireExist( output2, 1, 0*cosInvAngle - 0*sinInvAngle, 0*sinInvAngle + 0*cosInvAngle);
	RequireExist( output2, 0, 1*cosInvAngle - 0*sinInvAngle, 1*sinInvAngle + 0*cosInvAngle);
	RequireExist( output2, 0, 0*cosInvAngle - 1*sinInvAngle, 0*sinInvAngle + 1*cosInvAngle);
	RequireExist( output2, 1, 1*cosInvAngle - 1*sinInvAngle, 1*sinInvAngle + 1*cosInvAngle);
	RequireExist( output2, 1, 2*cosInvAngle - 3*sinInvAngle, 2*sinInvAngle + 3*cosInvAngle);

	//Changing Pose
	SetPosition(pose, 0, 0, 0);
	SetOrientation(pose, 0, std::sin(rotationAngle/2), 0, std::cos(rotationAngle/2));

	// Running DFN
	cartesianTransform->pointCloudInput(*cloud);
	cartesianTransform->poseInput(pose);
	cartesianTransform->process();
	const PointCloud& output3 = cartesianTransform->transformedPointCloudOutput();	

	//Testing Output
	REQUIRE( GetNumberOfPoints(output3) == 6);
	RequireExist( output3, 0*cosInvAngle + 0*sinInvAngle, 0, - 0*sinInvAngle + 0*cosInvAngle);
	RequireExist( output3, 1*cosInvAngle + 0*sinInvAngle, 0, - 1*sinInvAngle + 0*cosInvAngle);
	RequireExist( output3, 0*cosInvAngle + 0*sinInvAngle, 1, - 0*sinInvAngle + 0*cosInvAngle);
	RequireExist( output3, 0*cosInvAngle + 1*sinInvAngle, 0, - 0*sinInvAngle + 1*cosInvAngle);
	RequireExist( output3, 1*cosInvAngle + 1*sinInvAngle, 1, - 1*sinInvAngle + 1*cosInvAngle);
	RequireExist( output3, 1*cosInvAngle + 3*sinInvAngle, 2, - 1*sinInvAngle + 3*cosInvAngle);

	//Changing Pose
	SetPosition(pose, 0, 0, 0);
	SetOrientation(pose, 0, 0, std::sin(rotationAngle/2), std::cos(rotationAngle/2));

	// Running DFN
	cartesianTransform->pointCloudInput(*cloud);
	cartesianTransform->poseInput(pose);
	cartesianTransform->process();
	const PointCloud& output4 = cartesianTransform->transformedPointCloudOutput();	

	//Testing Output
	REQUIRE( GetNumberOfPoints(output4) == 6);
	RequireExist( output4, 0*cosInvAngle - 0*sinInvAngle, 0*sinInvAngle + 0*cosInvAngle, 0);
	RequireExist( output4, 1*cosInvAngle - 0*sinInvAngle, 1*sinInvAngle + 0*cosInvAngle, 0);
	RequireExist( output4, 0*cosInvAngle - 1*sinInvAngle, 0*sinInvAngle + 1*cosInvAngle, 0);
	RequireExist( output4, 0*cosInvAngle - 0*sinInvAngle, 0*sinInvAngle + 0*cosInvAngle, 1);
	RequireExist( output4, 1*cosInvAngle - 1*sinInvAngle, 1*sinInvAngle + 1*cosInvAngle, 1);
	RequireExist( output4, 1*cosInvAngle - 2*sinInvAngle, 1*sinInvAngle + 2*cosInvAngle, 3);

	//Changing Pose
	SetPosition(pose, 1, 2, 3);
	SetOrientation(pose, 0, 0, std::sin(rotationAngle/2), std::cos(rotationAngle/2));

	// Running DFN
	cartesianTransform->pointCloudInput(*cloud);
	cartesianTransform->poseInput(pose);
	cartesianTransform->process();
	const PointCloud& output5 = cartesianTransform->transformedPointCloudOutput();	

	//Testing Output
	REQUIRE( GetNumberOfPoints(output5) == 6);
	RequireExist( output5, 1, 2, 3);
	RequireExist( output5, 1 + 1*cosInvAngle - 0*sinInvAngle, 2 + 1*sinInvAngle + 0*cosInvAngle, 3);
	RequireExist( output5, 1 + 0*cosInvAngle - 1*sinInvAngle, 2 + 0*sinInvAngle + 1*cosInvAngle, 3);
	RequireExist( output5, 1 + 0*cosInvAngle - 0*sinInvAngle, 2 + 0*sinInvAngle + 0*cosInvAngle, 4);
	RequireExist( output5, 1 + 1*cosInvAngle - 1*sinInvAngle, 2 + 1*sinInvAngle + 1*cosInvAngle, 4);
	RequireExist( output5, 1 + 1*cosInvAngle - 2*sinInvAngle, 2 + 1*sinInvAngle + 2*cosInvAngle, 6);

	// Cleanup
	delete cartesianTransform;
	delete cloud;
}

TEST_CASE( "DFN configuration succeeds (CartesianSystemTransform)", "[configure]" )
{
	// Instantiate DFN
	CartesianSystemTransform* cartesianTransform = new CartesianSystemTransform;

	// Setup DFN
	cartesianTransform->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PointCloudTransform/CartesianSystemTransform_Conf1.yaml");
	cartesianTransform->configure();

	// Cleanup
	delete cartesianTransform;
}

/** @} */
