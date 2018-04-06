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
 * Testing application for the DFPC implementation FeaturesMatching3D.
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
#include <PointCloudModelLocalisation/FeaturesMatching3D.hpp>
#include <Errors/Assert.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <VisualPointFeatureVector3D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

#include <Converters/SupportTypes.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.cpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

#include <boost/make_shared.hpp>

using namespace dfpc_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;

int main(int argc, char** argv)
	{
	Visualizers::OpencvVisualizer::Enable();
	Visualizers::PclVisualizer::Enable();

	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);

	Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>* stubInputNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	Mocks::EigenTransformToTransform3DConverter* mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	WRITE_TO_LOG("Configure", "");
	FeaturesMatching3D featuresMatching3d;
	featuresMatching3d.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcFeaturesMatching3D_conf01.yaml");
	featuresMatching3d.setup();
	WRITE_TO_LOG("Configure", "Completed");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclModelCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *pclCloud);
	const unsigned SELECTION_RATIO = 10;
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
	featuresMatching3d.modelInput(modelCloud);
	WRITE_TO_LOG("Model Loaded", "");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclSceneCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(unsigned pointIndex = 0; pointIndex < pclModelCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pclModelCloud->points.at(pointIndex);
		pcl::PointXYZ scenePoint;
		scenePoint.x = point.x + 0.4;
		scenePoint.y = point.y + 0.4;
		scenePoint.z = point.z + 0.4;
		pclSceneCloud->points.push_back(scenePoint);
		}	
	for(float x = -0.1; x<0.1; x+=0.01)
		{
		for(float y = -0.1; y<0.1; y+=0.01)
			{
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, y, -0.1) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, y, 0.1) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(-0.1, x, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(0.1, x, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, -0.1, y) );
			pclSceneCloud->points.push_back( pcl::PointXYZ(x, 0.1, y) );
			}
		}
	PointCloudConstPtr sceneCloud = pclConverter.Convert(pclSceneCloud);
	featuresMatching3d.sceneInput(sceneCloud);
	WRITE_TO_LOG("Scene added in input", "");

	featuresMatching3d.run();

	bool success = featuresMatching3d.successOutput();
	Pose3DConstPtr pose = featuresMatching3d.poseOutput();
	if (!success)
		{
		WRITE_TO_LOG("No Pose found", "");
		ASSERT(pose == NULL, "Error, pose should be NULL if dfpc failed");
		}
	else
		{
		std::stringstream poseStream;
		poseStream << "Position: (" << GetXPosition(*pose) << ", " << GetYPosition(*pose) << ", " << GetZPosition(*pose) << ")" << "\n";
		poseStream << "Orientation: (" << GetXOrientation(*pose) << ", " << GetYOrientation(*pose) << ", " << GetZOrientation(*pose) << ", " << GetWOrientation(*pose) << ")" << "\n";
		std::string poseString = poseStream.str();
		WRITE_TO_LOG("Pose: \n", poseString);
		delete(pose);
		}

	PRINT_LOG();
	return 0;
	};

/** @} */
