/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Transform3DEigenTransformConvertersTest.cpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Eigen Transform to Transform3D and viceversa.
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
#include <Converters/Transform3DToEigenTransformConverter.hpp>
#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Types/CPP/Pose.hpp>
#include <Errors/Assert.hpp>
#include <boost/smart_ptr.hpp>
#include <Eigen/Geometry>

using namespace Converters;
using namespace PoseWrapper;
const double EPSILON = 0.000000001;

TEST_CASE( "Eigen Transform to Transform 3D and Back", "[EigenTransformToTransform]" )
	{
	EigenTransformToTransform3DConverter firstConverter;
	Transform3DToEigenTransformConverter secondConverter;

	Eigen::Matrix4f inputTransform;
	Eigen::Quaternionf inputRotation(std::cos(M_PI/2), std::sin(M_PI/2), std::sin(M_PI/2)/2, std::sin(M_PI/2)*2);
	Eigen::Matrix3f eigenRotationMatrix = inputRotation.normalized().toRotationMatrix();	
	inputTransform << 	eigenRotationMatrix(0,0), eigenRotationMatrix(0,1), eigenRotationMatrix(0,2), 0.3,
				eigenRotationMatrix(1,0), eigenRotationMatrix(1,1), eigenRotationMatrix(1,2), 0.4,
				eigenRotationMatrix(2,0), eigenRotationMatrix(2,1), eigenRotationMatrix(2,2), 0.6,
				0, 0, 0, 1;

	Transform3DSharedConstPtr asnTransform3D = firstConverter.ConvertShared(inputTransform);
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

	Eigen::Matrix4f outputTransform = secondConverter.ConvertShared(asnTransform3D);
	for(unsigned rowIndex = 0; rowIndex < 4; rowIndex++)
		{
		for(unsigned columnIndex = 0; columnIndex < 4; columnIndex++)
			{
			REQUIRE(outputTransform(rowIndex, columnIndex) < inputTransform(rowIndex, columnIndex) + EPSILON);
			REQUIRE(outputTransform(rowIndex, columnIndex) > inputTransform(rowIndex, columnIndex) - EPSILON);
			}
		}

	asnTransform3D.reset();
	} 

TEST_CASE( "Transform3D to Eigen Transform and Back", "[Transform3DToEigenTransform]" )
	{
	EigenTransformToTransform3DConverter firstConverter;
	Transform3DToEigenTransformConverter secondConverter;

	Eigen::Matrix4f inputTransform;
	Eigen::Quaternionf inputRotation(std::cos(M_PI/2), std::sin(M_PI/2), std::sin(M_PI/2)/2, std::sin(M_PI/2)*2);
	Eigen::Matrix3f eigenRotationMatrix = inputRotation.normalized().toRotationMatrix();	
	inputTransform << 	eigenRotationMatrix(0,0), eigenRotationMatrix(0,1), eigenRotationMatrix(0,2), 0.3,
				eigenRotationMatrix(1,0), eigenRotationMatrix(1,1), eigenRotationMatrix(1,2), 0.4,
				eigenRotationMatrix(2,0), eigenRotationMatrix(2,1), eigenRotationMatrix(2,2), 0.6,
				0, 0, 0, 1;

	Transform3DSharedConstPtr asnTransform3D = firstConverter.ConvertShared(inputTransform);
	Eigen::Matrix4f intermediateTransform = secondConverter.ConvertShared(asnTransform3D);
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

TEST_CASE("Bad Transform conversion 1 (Eigen Transform)", "[Bad Transform 1]")
	{
	EigenTransformToTransform3DConverter firstConverter;

	Eigen::Matrix4f inputTransform1;
	inputTransform1 << 0.3, 0.2, 0.1, 0.4, 0.3, 0.2, 0.1, 0.1, 0.5, 0.3, 0.2, 0.1, 1, 0, 0, 1;
	REQUIRE_THROWS( firstConverter.ConvertShared(inputTransform1) );
	}

TEST_CASE("Bad Transform conversion 2 (Eigen Transform)", "[Bad Transform 2]")
	{
	EigenTransformToTransform3DConverter firstConverter;

	Eigen::Matrix4f inputTransform2;
	inputTransform2 << 0.3, 0.2, 0.1, 0.4, 0.3, 0.2, 0.1, 0.1, 0.5, 0.3, 0.2, 0.1, 0, 0, 0, 1;
	REQUIRE_THROWS( firstConverter.ConvertShared(inputTransform2) );
	}
