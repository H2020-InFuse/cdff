/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Ransac3D.cpp
 * @date 17/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Ransac3D.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Catch/catch.h>
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

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

TEST_CASE( "Call to process", "[process]" ) 
	{
	Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache = 
		new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	//Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	//Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	//ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for(unsigned pointIndex = 0; pointIndex < 10; pointIndex++)
		{
		pointCloud->points.push_back(pcl::PointXYZ(0.1*pointIndex, -0.1*pointIndex, 0));
		FeatureType feature;
		feature.histogram[0] = 0.001*pointIndex;
		feature.histogram[1] = 0.002*pointIndex;
		for(int componentIndex = 0; componentIndex < MAX_FEATURES_NUMBER; componentIndex++)
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

	//VisualPointFeatureVector3DConstPtr featuresVector = new VisualPointFeatureVector3D();
	//mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

	VisualPointFeatureVector3DPtr sourceSet =  new VisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet =  new VisualPointFeatureVector3D();
	Ransac3D ransac;
	ransac.sourceFeaturesVectorInput(sourceSet);
	ransac.sinkFeaturesVectorInput(sinkSet);
	ransac.process();

	Transform3DConstPtr output = ransac.transformOutput();

	//REQUIRE(GetNumberOfCorrespondences(*output) == numberOfPoints);
	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	Ransac3D ransac;
	ransac.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/Ransac3D_Conf1.yaml");
	ransac.configure();	
	}

/** @} */
