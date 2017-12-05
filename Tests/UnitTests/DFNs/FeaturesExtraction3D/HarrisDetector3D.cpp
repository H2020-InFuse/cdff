/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN HarrisDetector3D.
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
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PclPointCloudToPointCloud3DConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloud3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace Common;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process", "[process]" ) 
	{
	Stubs::CacheHandler<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr>* stubInputCache = new Stubs::CacheHandler<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr>();
	Mocks::PointCloud3DToPclPointCloudConverter* mockInputConverter = new Mocks::PointCloud3DToPclPointCloudConverter();
	ConversionCache<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloud3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3D* >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3D*>();
	Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3D*, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	//Create a sample sphere
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud =  pcl::PointCloud<pcl::PointXYZ>::Ptr( new  pcl::PointCloud<pcl::PointXYZ>() );
	for(float alpha = 0; alpha < 2 * M_PI; alpha += 0.1)
		{
		for(float beta = 0; beta < 2*M_PI; beta += 0.1)
			{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			inputCloud->points.push_back(spherePoint);
			}
		}
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputCloud) );

	VisualPointFeatureVector3D* featuresVector = new VisualPointFeatureVector3D();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

	HarrisDetector3D harris;
	harris.process();

	VisualPointFeatureVector3D* output = harris.featuresSetOutput();

	REQUIRE(output->list.size == featuresVector->list.size);
	//No need to delete StubCacheHandler, MockImageTypeToMatConverter, StubCacheHandler, MockMatToVisualPointFeatureVector2DConverter
	//ConversionCache destructor takes care of that.
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	HarrisDetector3D harris;
	harris.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction3D/HarrisDetector3D_Conf1.yaml");
	harris.configure();	
	}

/** @} */
