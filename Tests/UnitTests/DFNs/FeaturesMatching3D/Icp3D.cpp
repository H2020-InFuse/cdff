/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Icp3D.cpp
 * @date 25/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Icp3D.
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
#include <FeaturesMatching3D/Icp3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process (3D ICP)", "[process]" ) 
	{
	Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache = 
		new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr >* stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	Mocks::EigenTransformToTransform3DConverter* mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for(unsigned pointIndex = 0; pointIndex < 10; pointIndex++)
		{
		pointCloud->points.push_back(pcl::PointXYZ(0.1*pointIndex, -0.1*pointIndex, 0));
		FeatureType feature;
		feature.histogram[0] = 0.001*pointIndex;
		feature.histogram[1] = 0.002*pointIndex;
		int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
		for(int componentIndex = 0; componentIndex < maxFeaturesNumber; componentIndex++)
			{
			feature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back(feature);
		}

	PointCloudWithFeatures sourceCloud;
	PointCloudWithFeatures sinkCloud;
	sourceCloud.pointCloud = pointCloud;
	sinkCloud.pointCloud = pointCloud;
	sourceCloud.featureCloud = featureCloud;
	sinkCloud.featureCloud = featureCloud;
	sourceCloud.descriptorSize = 2;
	sinkCloud.descriptorSize = 2;
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&sourceCloud) );
	mockInputConverter->AddBehaviour("Convert", "2", (void*) (&sinkCloud) );

	Transform3DPtr transform = new Transform3D();
	Reset(*transform);
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&transform) );

	VisualPointFeatureVector3DPtr sourceSet =  new VisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet =  new VisualPointFeatureVector3D();
	Icp3D icp;
	icp.sourceFeaturesVectorInput(sourceSet);
	icp.sinkFeaturesVectorInput(sinkSet);
	icp.process();

	Transform3DConstPtr output = icp.transformOutput();

	REQUIRE(GetXPosition(*output) == 0);
	REQUIRE(GetYPosition(*output) == 0);
	REQUIRE(GetZPosition(*output) == 0);
	REQUIRE(GetXOrientation(*output) == 0);
	REQUIRE(GetYOrientation(*output) == 0);
	REQUIRE(GetZOrientation(*output) == 0);
	REQUIRE(GetWOrientation(*output) == 0);
	delete(output);
	}

TEST_CASE( "Call to configure (3D ICP)", "[configure]" )
	{
	Icp3D icp;
	icp.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/Icp3D_Conf1.yaml");
	icp.configure();	
	}

/** @} */
