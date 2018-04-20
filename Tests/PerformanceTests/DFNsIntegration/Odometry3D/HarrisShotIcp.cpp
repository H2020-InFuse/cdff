/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisShotIcp.cpp
 * @date 20/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Harris 3d detector, Shot 3d descriptor, and Icp matcher 3d.
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <string>

#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.cpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <PclNormalsCloudToPointCloudConverter.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>
#include <PerformanceTests/Aggregator.hpp>

#include <Eigen/Geometry>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace SupportTypes;


class HarrisShotIcp : public PerformanceTestInterface
	{
	public:
		HarrisShotIcp(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName);
		~HarrisShotIcp();
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr >* stubNormalsCache;
		Mocks::PointCloudToPclNormalsCloudConverter* mockNormalsConverter;

		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubPointCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockPointCloudConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter;

		Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures >* stubFeaturesCloudCache;
		Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockFeaturesCloudConverter;

		Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubTransformCache;
		Mocks::EigenTransformToTransform3DConverter* mockTransformConverter;

		pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud;
		Pose3D groundTruthPose;

		PclPointCloudToPointCloudConverter pointCloudConverter;

		PointCloudConstPtr scenePointCloud;
		PointCloudConstPtr modelPointCloud;
		VisualPointFeatureVector3DConstPtr sceneKeypointsVector;
		VisualPointFeatureVector3DConstPtr modelKeypointsVector;
		VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		VisualPointFeatureVector3DConstPtr modelFeaturesVector;
		bool icpSuccess;
		Pose3DConstPtr modePoseInScene;

		HarrisDetector3D* harris;
		ShotDescriptor3D* shot;
		Icp3D* icp;

		Aggregator* groundPositionDistanceAggregator;
		Aggregator* groundOrientationDistanceAggregator;

		void LoadSceneCloud();
		void LoadModelCloud(int long inputId);

		void SetupMocksAndStubs();
		bool SetNextInputs();
		void ExecuteDfns();
		MeasuresMap ExtractMeasures();

		int long inputId;
	};

HarrisShotIcp::HarrisShotIcp(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	harris = new HarrisDetector3D();
	shot = new ShotDescriptor3D();
	icp = new Icp3D();
	AddDfn(harris);
	AddDfn(shot);
	AddDfn(icp);
	SetupMocksAndStubs();

	groundPositionDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("PositionDistance", groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	groundOrientationDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("AngleDistace", groundOrientationDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	scenePointCloud = NULL;
	modelPointCloud = NULL;
	sceneKeypointsVector = NULL;
	modelKeypointsVector = NULL;
	sceneFeaturesVector = NULL;
	modelFeaturesVector = NULL;
	modePoseInScene = NULL;
	icpSuccess = false;

	inputId = -1;
	LoadSceneCloud();
	}

HarrisShotIcp::~HarrisShotIcp()
	{
	delete(stubNormalsCache);
	delete(mockNormalsConverter);
	delete(stubPointCloudCache);
	delete(mockPointCloudConverter);
	delete(stubFeatures3dCache);
	delete(mockFeatures3dConverter);
	delete(stubFeaturesCloudCache);
	delete(mockFeaturesCloudConverter);
	delete(stubTransformCache);
	delete(mockTransformConverter);

	if (scenePointCloud != NULL)
		{
		delete(scenePointCloud);
		}
	if (modelPointCloud != NULL)
		{
		delete(modelPointCloud);
		}
	if (sceneKeypointsVector != NULL)
		{
		delete(sceneKeypointsVector);
		}
	if (modelKeypointsVector != NULL)
		{
		delete(modelKeypointsVector);
		}
	if (sceneFeaturesVector != NULL)
		{
		delete(sceneFeaturesVector);
		}
	if (modelFeaturesVector != NULL)
		{
		delete(modelFeaturesVector);
		}
	if (modePoseInScene != NULL)
		{
		delete(modePoseInScene);
		}
	}

void HarrisShotIcp::LoadSceneCloud()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../tests/Data/PointClouds/bunny0.ply", *basePclCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finitePointsPclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(unsigned pointIndex = 0; pointIndex < basePclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = basePclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			finitePointsPclCloud->points.push_back(point);
			}
		}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(finitePointsPclCloud);
	grid.setLeafSize(0.001, 0.001, 0.001);

	sceneCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	grid.filter(*sceneCloud);

	scenePointCloud = pointCloudConverter.Convert(sceneCloud);
	}

void HarrisShotIcp::LoadModelCloud(int long inputId)
	{
	if (modelPointCloud != NULL)
		{
		delete(modelPointCloud);
		}

	SetPosition(groundTruthPose, 0, 0, 0);
	SetOrientation(groundTruthPose, 0, 0, 0, 1);

	modelPointCloud = pointCloudConverter.Convert(sceneCloud);	
	}

void HarrisShotIcp::SetupMocksAndStubs()
	{
	stubNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	mockNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubNormalsCache, mockNormalsConverter);

	stubPointCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockPointCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubPointCloudCache, mockPointCloudConverter);

	stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	stubFeaturesCloudCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	mockFeaturesCloudConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubFeaturesCloudCache, mockFeaturesCloudConverter);

	stubTransformCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	mockTransformConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubTransformCache, mockTransformConverter);
	}

bool HarrisShotIcp::SetNextInputs()
	{
	inputId++;
	if (inputId > 0)
		{
		return false;
		}

	LoadModelCloud(inputId);
	return true;	
	}

void HarrisShotIcp::ExecuteDfns()
	{
	if (sceneKeypointsVector != NULL)
		{
		delete(sceneKeypointsVector);
		}
	harris->pointCloudInput(scenePointCloud);
	harris->process();
	sceneKeypointsVector = harris->featuresSetOutput();

	if (modelKeypointsVector != NULL)
		{
		delete(modelKeypointsVector);
		}
	harris->pointCloudInput(modelPointCloud);
	harris->process();
	modelKeypointsVector = harris->featuresSetOutput();

	if (sceneFeaturesVector != NULL)
		{
		delete(sceneFeaturesVector);
		}
	shot->pointCloudInput(scenePointCloud);
	shot->featuresSetInput(sceneKeypointsVector);
	shot->process();
	sceneFeaturesVector = shot->featuresSetWithDescriptorsOutput();

	if (modelFeaturesVector != NULL)
		{
		delete(modelFeaturesVector);
		}
	shot->pointCloudInput(modelPointCloud);
	shot->featuresSetInput(modelKeypointsVector);
	shot->process();
	modelFeaturesVector = shot->featuresSetWithDescriptorsOutput();

	if (modePoseInScene != NULL)
		{
		delete(modePoseInScene);
		}
	icp->sourceFeaturesVectorInput(modelFeaturesVector);
	icp->sinkFeaturesVectorInput(sceneFeaturesVector);
	icp->process();
	icpSuccess = icp->successOutput();
	modePoseInScene = icp->transformOutput();
	}

HarrisShotIcp::MeasuresMap HarrisShotIcp::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["IcpSuccess"] = icpSuccess;
	measuresMap["SceneKeypoints"] = GetNumberOfPoints(*sceneKeypointsVector);
	measuresMap["ModelKeypoints"] = GetNumberOfPoints(*modelKeypointsVector);

	if (icpSuccess)
		{
		float differenceX = GetXPosition(groundTruthPose) - GetXPosition(*modePoseInScene);
		float differenceY = GetYPosition(groundTruthPose) - GetYPosition(*modePoseInScene);
		float differenceZ = GetZPosition(groundTruthPose) - GetZPosition(*modePoseInScene);
		float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;
		measuresMap["PositionDistance"] = std::sqrt(squaredDistance);

		float scalarProduct =
			GetXOrientation(groundTruthPose) * GetXOrientation(*modePoseInScene) +
			GetYOrientation(groundTruthPose) * GetYOrientation(*modePoseInScene) +
			GetZOrientation(groundTruthPose) * GetZOrientation(*modePoseInScene) +
			GetWOrientation(groundTruthPose) * GetWOrientation(*modePoseInScene);
		measuresMap["AngleDistace"] = 1 - scalarProduct*scalarProduct;				
		}
	else
		{
		measuresMap["PositionDistance"] = 1;
		measuresMap["AngleDistace"] = 1;
		}

	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::vector<std::string> baseConfigurationFiles =
		{
		"HarrisDetector3d_PerformanceTest_1.yaml",
		"ShotDescriptor3d_PerformanceTest_1.yaml",
		"Icp3d_PerformanceTest_1.yaml"
		};
	HarrisShotIcp interface("../tests/ConfigurationFiles/DFNsIntegration/Odometry3D", baseConfigurationFiles, "Harris_Shot_Icp.txt");
	interface.Run();
	};

/** @} */
