/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ObservedScene.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ObservedScene class.
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
#include "ObservedScene.hpp"
#include "Errors/Assert.hpp"
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Transform3DToEigenTransformConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>

namespace dfpc_ci {

using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ObservedScene::ObservedScene() : 
	scene( pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() ) )
	{

	}

ObservedScene::~ObservedScene()
	{

	}

void ObservedScene::AddFrame(FrameConstPtr frame)
	{
	FrameNode newNode;
	newNode.frame = frame;
	newNode.transformInOrigin = IdentityTransform();
	newNode.validTransform = (framesMap.size() == 0);	
	framesMap.push_back(newNode);
	referenceFrameId = framesMap.size() - 1;
	}

FrameConstPtr ObservedScene::GetNextReferenceFrame()
	{
	while(referenceFrameId > 0)
		{
		referenceFrameId--;
		if (framesMap.at(referenceFrameId).validTransform)
			{
			return framesMap.at(referenceFrameId).frame;
			}
		}
	return NULL;
	}

void ObservedScene::AddFramePose(PoseWrapper::Pose3DConstPtr poseInReference)
	{
	framesMap.at( framesMap.size()-1 ).transformInOrigin = Convert(poseInReference);
	framesMap.at( framesMap.size()-1 ).validTransform = true;
	}

void ObservedScene::AddPointCloudInLastReference(PointCloudConstPtr pointCloudInReference)
	{
	AffineTransform transformInOrigin = framesMap.at(referenceFrameId).transformInOrigin;

	for(unsigned pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloudInReference); pointIndex++)
		{
		pcl::PointXYZ point( GetXCoordinate(*pointCloudInReference, pointIndex), GetYCoordinate(*pointCloudInReference, pointIndex), GetZCoordinate(*pointCloudInReference, pointIndex) );
		pcl::PointXYZ pointInOrigin = TransformPoint(point, transformInOrigin);
		scene->points.push_back(pointInOrigin);
		}
	}

PointCloudConstPtr ObservedScene::GetPartialScene(Point3D origin, float radius)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr partialScene(new pcl::PointCloud<pcl::PointXYZ>() );
	
	for(unsigned pointIndex = 0; pointIndex < scene->points.size(); pointIndex++)
		{
		pcl::PointXYZ currentPoint = scene->points.at(pointIndex);
		float distanceX = currentPoint.x - origin.x;
		float distanceY = currentPoint.y - origin.y;
		float distanceZ = currentPoint.z - origin.z;
		float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;

		if (squaredDistance < radius*radius)
			{
			partialScene->points.push_back(currentPoint);
			}
		}

	return ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Convert(partialScene);
	}

PointCloudWrapper::PointCloudConstPtr ObservedScene::GetPartialScene(float radius)
	{
	AffineTransform currentPose = framesMap.at(framesMap.size()-1).transformInOrigin;
	Eigen::Vector3f currentPosition = currentPose.translation();
	
	Point3D origin;
	origin.x = currentPosition.x();
	origin.x = currentPosition.y();
	origin.x = currentPosition.z();

	return GetPartialScene(origin, radius);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
ObservedScene::AffineTransform ObservedScene::IdentityTransform()
	{
	Eigen::Quaternion<float> rotation(1, 0, 0, 0);
	Eigen::Translation<float, 3> translation( 0, 0, 0);
	AffineTransform affineTransform = translation * rotation;
	return affineTransform;
	}

ObservedScene::AffineTransform ObservedScene::Convert(PoseWrapper::Transform3DConstPtr transform)
	{
	Eigen::Quaternion<float> rotation(GetWRotation(*transform), GetXRotation(*transform), GetYRotation(*transform), GetZRotation(*transform));
	Eigen::Translation<float, 3> translation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));
	AffineTransform affineTransform = translation * rotation;
	return affineTransform;
	}

pcl::PointXYZ ObservedScene::TransformPoint(pcl::PointXYZ point, AffineTransform transform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = transform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

}
/** @} */
