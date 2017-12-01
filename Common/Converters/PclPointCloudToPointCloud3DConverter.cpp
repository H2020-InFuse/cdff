/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloud3DConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PclPointCloudToPointCloud3DConverter class.
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

#include "PclPointCloudToPointCloud3DConverter.hpp"
#include <Errors/Assert.hpp>
#include <stdio.h>
#include <math.h>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloud3D* PclPointCloudToPointCloud3DConverter::Convert(const  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
	{
	ASSERT(true, "PclPointCloudToPointCloud3DConverter: point cloud type not supported yet");

	PointCloud3D* asnPointCloud = new PointCloud3D();
	asnPointCloud->size = pointCloud->points.size();
	asnPointCloud->point_cloud_mode = PointCloud3DMode_base_mode;
	
	for(int pointIndex; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		int error;
		
		double* xCoordinate = new double( pointCloud->points.at(pointIndex).x );
		
		error = ASN_SEQUENCE_ADD( &(asnPointCloud->data.list.array), xCoordinate  );
		ASSERT(error == 0, "PclPointCloudToPointCloud3DConverter, conversion failed");

		double* yCoordinate = new double( pointCloud->points.at(pointIndex).y );
		error = ASN_SEQUENCE_ADD( &(asnPointCloud->data.list.array), yCoordinate  );
		ASSERT(error == 0, "PclPointCloudToPointCloud3DConverter, conversion failed");

		double* zCoordinate = new double( pointCloud->points.at(pointIndex).z );
		error = ASN_SEQUENCE_ADD( &(asnPointCloud->data.list.array), zCoordinate );
		ASSERT(error == 0, "PclPointCloudToPointCloud3DConverter, conversion failed");	
		}
	
	PRINT_TO_LOG("out:", asnPointCloud->data.list.count);
	PRINT_TO_LOG("in:", pointCloud->points.size());
	return asnPointCloud;
	}

}

/** @} */
