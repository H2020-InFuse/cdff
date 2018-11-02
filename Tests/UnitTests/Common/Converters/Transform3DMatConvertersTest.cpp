/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DMatConvertersTest.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from cv Matrix to Transform3D and viceversa.
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
#include <Converters/Transform3DToMatConverter.hpp>
#include <Converters/MatToTransform3DConverter.hpp>
#include <Pose.hpp>
#include <Errors/Assert.hpp>
#include <boost/smart_ptr.hpp>
#include <Eigen/Geometry>

using namespace Converters;
using namespace PoseWrapper;
const double EPSILON = 0.000000001;

TEST_CASE( "Mat to Transform 3D and Back", "[MatToTransform]" )
	{
	MatToTransform3DConverter firstConverter;
	Transform3DToMatConverter secondConverter;

	Eigen::Matrix4f inputTransform;
	Eigen::Quaternionf inputRotation(std::cos(M_PI/2), std::sin(M_PI/2), std::sin(M_PI/2)/2, std::sin(M_PI/2)*2);
	Eigen::Matrix3f eigenRotationMatrix = inputRotation.normalized().toRotationMatrix();	
	inputTransform << 	eigenRotationMatrix(0,0), eigenRotationMatrix(0,1), eigenRotationMatrix(0,2), 0.3,
				eigenRotationMatrix(1,0), eigenRotationMatrix(1,1), eigenRotationMatrix(1,2), 0.4,
				eigenRotationMatrix(2,0), eigenRotationMatrix(2,1), eigenRotationMatrix(2,2), 0.6,
				0, 0, 0, 1;

	cv::Mat cvInputTransform(3, 4, CV_32FC1);
	for(unsigned row = 0; row<3; row++)
		{
		for(unsigned column = 0; column < 4; column++)
			{
			cvInputTransform.at<float>(row, column) = inputTransform(row, column);
			}
		}

	Transform3DSharedConstPtr asnTransform3D = firstConverter.ConvertShared(cvInputTransform);
	REQUIRE( GetXPosition(*asnTransform3D) < inputTransform(0, 3) + EPSILON );
	REQUIRE( GetXPosition(*asnTransform3D) > inputTransform(0, 3) - EPSILON );
	REQUIRE( GetYPosition(*asnTransform3D) < inputTransform(1, 3) + EPSILON );
	REQUIRE( GetYPosition(*asnTransform3D) > inputTransform(1, 3) - EPSILON );
	REQUIRE( GetZPosition(*asnTransform3D) < inputTransform(2, 3) + EPSILON );
	REQUIRE( GetZPosition(*asnTransform3D) > inputTransform(2, 3) - EPSILON );
	REQUIRE( GetXOrientation(*asnTransform3D) < inputRotation.normalized().x() + EPSILON );
	REQUIRE( GetXOrientation(*asnTransform3D) > inputRotation.normalized().x() - EPSILON );
	REQUIRE( GetYOrientation(*asnTransform3D) < inputRotation.normalized().y() + EPSILON );
	REQUIRE( GetYOrientation(*asnTransform3D) > inputRotation.normalized().y() - EPSILON );
	REQUIRE( GetZOrientation(*asnTransform3D) < inputRotation.normalized().z() + EPSILON );
	REQUIRE( GetZOrientation(*asnTransform3D) > inputRotation.normalized().z() - EPSILON );
	REQUIRE( GetWOrientation(*asnTransform3D) < inputRotation.normalized().w() + EPSILON );
	REQUIRE( GetWOrientation(*asnTransform3D) > inputRotation.normalized().w() - EPSILON );

	cv::Mat outputTransform = secondConverter.ConvertShared(asnTransform3D);
	for(unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for(unsigned columnIndex = 0; columnIndex < 4; columnIndex++)
			{
			REQUIRE(outputTransform.at<float>(rowIndex, columnIndex) < inputTransform(rowIndex, columnIndex) + EPSILON);
			REQUIRE(outputTransform.at<float>(rowIndex, columnIndex) > inputTransform(rowIndex, columnIndex) - EPSILON);
			}
		}

	asnTransform3D.reset();
	} 

TEST_CASE( "Transform3D to Mat and Back", "[Transform3DToMat]" )
	{
	MatToTransform3DConverter firstConverter;
	Transform3DToMatConverter secondConverter;

	Eigen::Matrix4f inputTransform;
	Eigen::Quaternionf inputRotation(std::cos(M_PI/2), std::sin(M_PI/2), std::sin(M_PI/2)/2, std::sin(M_PI/2)*2);
	Eigen::Matrix3f eigenRotationMatrix = inputRotation.normalized().toRotationMatrix();	
	inputTransform << 	eigenRotationMatrix(0,0), eigenRotationMatrix(0,1), eigenRotationMatrix(0,2), 0.3,
				eigenRotationMatrix(1,0), eigenRotationMatrix(1,1), eigenRotationMatrix(1,2), 0.4,
				eigenRotationMatrix(2,0), eigenRotationMatrix(2,1), eigenRotationMatrix(2,2), 0.6,
				0, 0, 0, 1;

	cv::Mat cvInputTransform(3, 4, CV_32FC1);
	for(unsigned row = 0; row<3; row++)
		{
		for(unsigned column = 0; column < 4; column++)
			{
			cvInputTransform.at<float>(row, column) = inputTransform(row, column);
			}
		}

	Transform3DSharedConstPtr asnTransform3D = firstConverter.ConvertShared(cvInputTransform);
	cv::Mat intermediateTransform = secondConverter.ConvertShared(asnTransform3D);
	Transform3DSharedConstPtr outputTransform = firstConverter.ConvertShared(intermediateTransform);

	REQUIRE( GetXPosition(*asnTransform3D) < GetXPosition(*outputTransform) + EPSILON );
	REQUIRE( GetXPosition(*asnTransform3D) > GetXPosition(*outputTransform) - EPSILON );
	REQUIRE( GetYPosition(*asnTransform3D) < GetYPosition(*outputTransform) + EPSILON );
	REQUIRE( GetYPosition(*asnTransform3D) > GetYPosition(*outputTransform) - EPSILON );
	REQUIRE( GetZPosition(*asnTransform3D) < GetZPosition(*outputTransform) + EPSILON );
	REQUIRE( GetZPosition(*asnTransform3D) > GetZPosition(*outputTransform) - EPSILON );
	REQUIRE( GetXOrientation(*asnTransform3D) < GetXOrientation(*outputTransform) + EPSILON );
	REQUIRE( GetXOrientation(*asnTransform3D) > GetXOrientation(*outputTransform) - EPSILON );
	REQUIRE( GetYOrientation(*asnTransform3D) < GetYOrientation(*outputTransform) + EPSILON );
	REQUIRE( GetYOrientation(*asnTransform3D) > GetYOrientation(*outputTransform) - EPSILON );
	REQUIRE( GetZOrientation(*asnTransform3D) < GetZOrientation(*outputTransform) + EPSILON );
	REQUIRE( GetZOrientation(*asnTransform3D) > GetZOrientation(*outputTransform) - EPSILON );
	REQUIRE( GetWOrientation(*asnTransform3D) < GetWOrientation(*outputTransform) + EPSILON );
	REQUIRE( GetWOrientation(*asnTransform3D) > GetWOrientation(*outputTransform) - EPSILON );

	asnTransform3D.reset();
	outputTransform.reset();
	} 

TEST_CASE("Bad Transform conversion 1 (Mat)", "[Bad Transform 1]")
	{
	MatToTransform3DConverter firstConverter;

	Eigen::Matrix4f inputTransform;
	inputTransform << 0.3, 0.2, 0.1, 0.4, 0.3, 0.2, 0.1, 0.1, 0.5, 0.3, 0.2, 0.1, 1, 0, 0, 1;

	cv::Mat cvInputTransform(3, 4, CV_32FC1);
	for(unsigned row = 0; row<3; row++)
		{
		for(unsigned column = 0; column < 4; column++)
			{
			cvInputTransform.at<float>(row, column) = inputTransform(row, column);
			}
		}

	REQUIRE_THROWS( firstConverter.ConvertShared(cvInputTransform) );
	}

TEST_CASE("Bad Transform conversion 2 (Mat)", "[Bad Transform 2]")
	{
	MatToTransform3DConverter firstConverter;

	Eigen::Matrix4f inputTransform;
	inputTransform << 0.3, 0.2, 0.1, 0.4, 0.3, 0.2, 0.1, 0.1, 0.5, 0.3, 0.2, 0.1, 0, 0, 0, 1;

	cv::Mat cvInputTransform(3, 4, CV_32FC1);
	for(unsigned row = 0; row<3; row++)
		{
		for(unsigned column = 0; column < 4; column++)
			{
			cvInputTransform.at<float>(row, column) = inputTransform(row, column);
			}
		}

	REQUIRE_THROWS( firstConverter.ConvertShared(cvInputTransform) );
	}
