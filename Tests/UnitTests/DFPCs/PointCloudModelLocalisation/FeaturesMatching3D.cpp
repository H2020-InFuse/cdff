/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFPCs FeaturesMatching3D.
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
#include <PointCloudModelLocalisation/FeaturesMatching3D.hpp>
#include <Errors/Assert.hpp>

#include<pcl/io/ply_io.h>
#include <Types/CPP/Pose.hpp>

#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/SupportTypes.hpp>

#include <boost/make_shared.hpp>


using namespace CDFF::DFPC::PointCloudModelLocalisation;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Success Call to Configure (3D feature registration)", "[configureSuccess]" ) 
	{
	FeaturesMatching3D* featuresMatching3d = new FeaturesMatching3D();
	featuresMatching3d->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcFeaturesMatching3D_conf01.yaml");
	featuresMatching3d->setup();
	delete(featuresMatching3d);
	}

TEST_CASE( "Success Call to Process (3D feature registration)", "[processSuccess]" ) 
	{
	FeaturesMatching3D* featuresMatching3d = new FeaturesMatching3D();
	featuresMatching3d->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcFeaturesMatching3D_conf01.yaml");
	featuresMatching3d->setup();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclModelCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../tests/Data/PointClouds/bunny0.ply", *pclCloud);
	const unsigned SELECTION_RATIO = 1000;
	unsigned selectionCounter = 0;
	for(unsigned pointIndex = 0; pointIndex < pclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			if (selectionCounter == 0)
				{
				pclModelCloud->points.push_back(point);
				}
			selectionCounter = (selectionCounter+1)%SELECTION_RATIO;
			}
		}
	PclPointCloudToPointCloudConverter pclConverter;
	PointCloudConstPtr modelCloud = pclConverter.Convert(pclModelCloud);
	featuresMatching3d->modelInput(*modelCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclSceneCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(unsigned pointIndex = 0; pointIndex < pclModelCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pclModelCloud->points.at(pointIndex);
		pcl::PointXYZ scenePoint;
		scenePoint.x = point.x + 0.2;
		scenePoint.y = point.y - 0.1;
		scenePoint.z = point.z + 0.2;
		pclSceneCloud->points.push_back(scenePoint);
		}	
	for(float x = -1; x<1; x+=0.1)
		{
		for(float y = -1; y<1; y+=0.1)
			{
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, y, -1) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, y, 1) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(-1, x, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(1, x, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, -1, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, 1, y) );
			}
		}
	PointCloudConstPtr sceneCloud = pclConverter.Convert(pclSceneCloud);
	featuresMatching3d->sceneInput(*sceneCloud);
	WRITE_TO_LOG("Scene added in input", "");

	featuresMatching3d->run();

	bool success = featuresMatching3d->successOutput();
	const Pose3D& pose = featuresMatching3d->poseOutput();

	delete(modelCloud);
	delete(sceneCloud);
	delete(featuresMatching3d);
	}


/** @} */
