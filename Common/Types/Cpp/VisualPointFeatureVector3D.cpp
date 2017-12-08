/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3D.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of VisualPointFeatureVector3D class.
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
#include "VisualPointFeatureVector3D.hpp"
#include <Errors/Assert.hpp>

namespace CppTypes
{


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

VisualPointFeatureVector3D::VisualPointFeatureVector3D()
	{
	ClearPoints();
	}

VisualPointFeatureVector3D::~VisualPointFeatureVector3D()
	{

	}

void VisualPointFeatureVector3D::AddPoint(float x, float y, float z)
	{
	ASSERT(featuresVector.nCount < MAX_FEATURE_3D_POINTS, "Features descriptor vector maximum capacity has been reached");
	int currentIndex = featuresVector.nCount;
	featuresVector.arr[currentIndex].point.x = x;
	featuresVector.arr[currentIndex].point.y = y;	
	featuresVector.arr[currentIndex].point.z = z;	
	featuresVector.arr[currentIndex].descriptor.nCount = 0;
	featuresVector.nCount++;
	}

void VisualPointFeatureVector3D::ClearPoints()
	{
	featuresVector.nCount = 0;
	}

int VisualPointFeatureVector3D::GetNumberOfPoints() const
	{
	return featuresVector.nCount;
	}

float VisualPointFeatureVector3D::GetXCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.x;
	}

float VisualPointFeatureVector3D::GetYCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.y;
	}

float VisualPointFeatureVector3D::GetZCoordinate(int pointIndex) const
	{
	ASSERT(pointIndex < featuresVector.nCount, "A missing point was requested from a features vector 2D");
	return featuresVector.arr[pointIndex].point.z;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const T_UInt32 VisualPointFeatureVector3D::MAX_FEATURE_3D_POINTS = CTypes::features3DElementsMax;
const T_UInt32 VisualPointFeatureVector3D::MAX_DESCRIPTOR_3D_LENGTH = CTypes::descriptor3DNameLength;


}

/** @} */
