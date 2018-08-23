/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestPointCloudModelLocalisation.hpp
 * @date 29/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This is the main interface class for implementations of the DFPC Point Cloud Model Localisation.
 * It performs some common operations
 *
 * @{
 */

#ifndef GUI_TEST_POINT_CLOUD_MODEL_LOCALISATION_HPP
#define GUI_TEST_POINT_CLOUD_MODEL_LOCALISATION_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
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

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class GuiTestPointCloudModelLocalisation
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		GuiTestPointCloudModelLocalisation(std::string configurationFilePath, std::string sceneFilePath, std::string modelFilePath);
		~GuiTestPointCloudModelLocalisation();

		void Run(CDFF::DFPC::PointCloudModelLocalisationInterface& detector3d);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

		Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache;
		Mocks::MatToPose3DConverter* mockEssentialPoseConverter;

		Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache;
		Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter;

		Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache;
		Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter;

		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>* stubInputNormalsCache;
		Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter;

		Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache;
		Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter;

		Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache;
		Mocks::EigenTransformToTransform3DConverter* mockOutputConverter;

		std::string modelFilePath;
		std::string sceneFilePath;
		std::string configurationFilePath;

		Converters::PclPointCloudToPointCloudConverter frameConverter;

		void SetUpMocksAndStubs();
		void LoadPointClouds(PointCloudWrapper::PointCloudConstPtr& scene, PointCloudWrapper::PointCloudConstPtr& model);

	};

#endif

/* GuiTestPointCloudModelLocalisation.hpp */
/** @} */
