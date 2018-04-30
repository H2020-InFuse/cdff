/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclVisualizer.cpp
 * @date 28/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Visualizers
 * 
 * Implementation of the PclVisualizer class
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
#include "PclVisualizer.hpp"
#include <Errors/Assert.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <pcl/io/ply_io.h>

using namespace PointCloudWrapper;
using namespace Converters;
using namespace Common;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;

#define RETURN_IF_DISABLED \
	if (!enabled) \
		{ \
		return; \
		}

#define RETURN_IF_SAVING_DISABLED \
	if (!enabledSaving) \
		{ \
		return; \
		}

namespace Visualizers
{
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PclVisualizer::ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
	{
	RETURN_IF_DISABLED
	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList;
	pointCloudsList.push_back(pointCloud);	

	ShowPointClouds(pointCloudsList);
	}

void PclVisualizer::ShowPointClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList)
	{
	RETURN_IF_DISABLED
	pcl::visualization::PCLVisualizer viewer (WINDOW_NAME);
	ASSERT( pointCloudsList.size() <= MAX_POINT_CLOUDS, "PclVisualizer, too many point cloud to visualize");
	viewer.addCoordinateSystem(0.1);

	for(unsigned pointCloudIndex = 0; pointCloudIndex < pointCloudsList.size(); pointCloudIndex++)
		{
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud = pointCloudsList.at(pointCloudIndex);
		Color cloudColor = COLORS_LIST[pointCloudIndex];
    		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pointCloud, cloudColor.r, cloudColor.g, cloudColor.b);

		std::stringstream cloudId;
		cloudId << "Point Cloud "<<pointCloudIndex;
    		viewer.addPointCloud(pointCloud, pclCloudColor, cloudId.str());
		}

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	viewer.removeAllPointClouds();
	viewer.close();
	}

void PclVisualizer::ShowPointCloud(PointCloudConstPtr pointCloud)
	{
	RETURN_IF_DISABLED
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(pointCloud);
	ShowPointCloud(pclPointCloud);
	}

void PclVisualizer::ShowVisualFeatures(PointCloudConstPtr pointCloud, VisualPointFeatureVector3DConstPtr featuresVector)
	{
	RETURN_IF_DISABLED
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(pointCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr featuresCloud(new pcl::PointCloud<pcl::PointXYZ>() );

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		VisualPointType type = GetPointType(*featuresVector, pointIndex);
		pcl::PointXYZ point;
		if (type == VISUAL_POINT_POSITION)
			{
			point.x = GetXCoordinate(*featuresVector, pointIndex);
			point.y = GetYCoordinate(*featuresVector, pointIndex);
			point.z = GetZCoordinate(*featuresVector, pointIndex);
			}
		else if (type == VISUAL_POINT_REFERENCE)
			{
			point = pclPointCloud->points.at( GetReferenceIndex(*featuresVector, pointIndex) );
			}
		else
			{
			ASSERT(false, "PclVisualizer, unhandled point type in VisualPointFeatureVector3D");
			}
		featuresCloud->points.push_back(point);
		}

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList;
	pointCloudsList.push_back(pclPointCloud);
	pointCloudsList.push_back(featuresCloud);
	
	ShowPointClouds(pointCloudsList);
	}

void PclVisualizer::ShowImage(pcl::PointCloud<pcl::RGB>::ConstPtr image)
	{
	RETURN_IF_DISABLED
	pcl::visualization::PCLVisualizer viewer (WINDOW_NAME);
	viewer.addCoordinateSystem(0.1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
	for(unsigned row = 0; row < image->height; row++)
		{
		for(unsigned column = 0; column < image->width; column++)
			{
			pcl::RGB point = image->points.at(row * image->width + column);
			pcl::PointXYZRGB imageCloudPoint;
			imageCloudPoint.x = static_cast<float>(column) * 0.01;
			imageCloudPoint.y = - static_cast<float>(row) * 0.01;
			imageCloudPoint.z = 0;
			imageCloudPoint.r = point.r;
			imageCloudPoint.g = point.g;
			imageCloudPoint.b = point.b;
			imageCloud->points.push_back(imageCloudPoint);
			}
		}
	viewer.addPointCloud<pcl::PointXYZRGB>(imageCloud);

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	viewer.removeAllPointClouds();
	viewer.close();
	}

void PclVisualizer::PlacePointCloud(PointCloudConstPtr sceneCloud, PointCloudConstPtr objectCloud, Pose3DConstPtr objectPoseInScene)
	{
	RETURN_IF_DISABLED
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclSceneCloud=ConversionCache<PointCloudConstPtr,pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(sceneCloud);	
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclObjectCloud=ConversionCache<PointCloudConstPtr,pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(objectCloud);	

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedObject(new pcl::PointCloud<pcl::PointXYZ>() );
	for(unsigned pointIndex = 0; pointIndex < pclObjectCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPoint( pclObjectCloud->points.at(pointIndex), objectPoseInScene );
		transformedObject->points.push_back(transformedPoint);
		}

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointsCloudList;
	pointsCloudList.push_back(pclSceneCloud);
	pointsCloudList.push_back(transformedObject);
	ShowPointClouds(pointsCloudList);
	}

void PclVisualizer::SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, unsigned period)
	{
	static unsigned time = 1;
	RETURN_IF_SAVING_DISABLED
	if (time % period != 0)
		{
		return;
		}

	std::stringstream cloudOutputPath;
	cloudOutputPath << SAVE_FILE_BASE_NAME << time << SAVE_FILE_EXTENSION;

	pcl::PLYWriter writer;
	writer.write(cloudOutputPath.str(), *pointCloud);

	time++;
	}
	
void PclVisualizer::SavePointCloud(PointCloudConstPtr pointCloud, unsigned period)
	{
	RETURN_IF_SAVING_DISABLED
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(pointCloud);
	SavePointCloud(pclPointCloud, period);
	}

void PclVisualizer::Enable()
	{
	enabled = true;
	}

void PclVisualizer::Disable()
	{
	enabled = false;
	}

void PclVisualizer::EnableSaving()
	{
	enabledSaving = true;
	}

void PclVisualizer::DisableSaving()
	{
	enabledSaving = false;
	}

/* --------------------------------------------------------------------------
 *
 * Protected Member Functions
 *
 * --------------------------------------------------------------------------
 */
PclVisualizer::PclVisualizer()
	{

	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string PclVisualizer::WINDOW_NAME = "PCL Visualizer";
const std::string PclVisualizer::SAVE_FILE_BASE_NAME = "VisualizerCloud";
const std::string PclVisualizer::SAVE_FILE_EXTENSION = ".ply";
const PclVisualizer::Color PclVisualizer::COLORS_LIST[MAX_POINT_CLOUDS] = 
	{
	{.r = 255, .g = 255, .b = 255},
	{.r = 200, .g = 0, .b = 0},
	{.r = 0, .g = 200, .b = 0},
	{.r = 0, .g = 0, .b = 200},
	{.r = 200, .g = 200, .b = 0},
	{.r = 200, .g = 0, .b = 200},
	{.r = 0, .g = 200, .b = 200},
	{.r = 200, .g = 100, .b = 100},
	{.r = 100, .g = 200, .b = 100},
	{.r = 100, .g = 100, .b = 200}
	};
bool PclVisualizer::enabled = false;
bool PclVisualizer::enabledSaving = false;


/* --------------------------------------------------------------------------
 *
 * Private Member Methods
 *
 * --------------------------------------------------------------------------
 */
pcl::PointXYZ PclVisualizer::TransformPoint(pcl::PointXYZ point, Transform3DConstPtr transform)
	{
	Eigen::Quaternion<float> rotation(GetWRotation(*transform), GetXRotation(*transform), GetYRotation(*transform), GetZRotation(*transform));
	Eigen::Translation<float, 3> translation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> affineTransform = translation * rotation;

	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}
}
/** @} */

