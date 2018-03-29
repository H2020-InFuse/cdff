/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestPointCloudModelLocalisation.cpp
 * @date 29/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the GuiTestPointCloudModelLocalisation class.
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
#include "GuiTestPointCloudModelLocalisation.hpp"

using namespace dfpc_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
GuiTestPointCloudModelLocalisation::GuiTestPointCloudModelLocalisation(std::string configurationFilePath, std::string sceneFilePath, std::string modelFilePath) 
	{
	this->configurationFilePath = configurationFilePath;
	this->sceneFilePath = sceneFilePath;
	this->modelFilePath = modelFilePath;

	SetUpMocksAndStubs();
	}

GuiTestPointCloudModelLocalisation::~GuiTestPointCloudModelLocalisation()
	{
	delete(stubEssentialPoseCache);
	delete(mockEssentialPoseConverter);

	delete(stubTriangulationPoseCache);
	delete(mockTriangulationPoseConverter);

	delete(stubCloudCache);
	delete(mockCloudConverter);

	delete(stubInverseCloudCache);
	delete(mockInverseCloudConverter);

	delete(stubFeatures3dCache);
	delete(mockFeatures3dConverter);

	delete(stubInputNormalsCache);
	delete(mockInputNormalsConverter);

	delete(stubInputCache);
	delete(mockInputConverter);

	delete(stubOutputCache);
	delete(mockOutputConverter);
	}

void GuiTestPointCloudModelLocalisation::Run(dfpc_ci::PointCloudModelLocalisationInterface& detector3d)
	{
	detector3d.setConfigurationFile(configurationFilePath);
	detector3d.setup();

	PointCloudConstPtr scene, model;
	LoadNextPointCloud(scene, model)

	detector3d.sceneInput(scene);
	detector3d.modelInput(model);
	detector3d.run();

	Pose3DConstPtr pose = detector3d.poseOutput();
	bool success = detector3d.successOutput();

	delete(scene);
	delete(model);

	if (pose != NULL)
		{
		delete(pose);
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void GuiTestPointCloudModelLocalisation::SetUpMocksAndStubs()
	{
	stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);

	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	stubInputNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void GuiTestPointCloudModelLocalisation::LoadPointClouds(PointCloudWrapper::PointCloudConstPtr& scene, PointCloudWrapper::PointCloudConstPtr& model)
	{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclScene(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile(sceneFilePath, *pclScene);
	scene = converter.Convert(pclScene);

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclModel(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile(modelFilePath, *pclModel);
	model = converter.Convert(pclModel);
	}

/** @} */
