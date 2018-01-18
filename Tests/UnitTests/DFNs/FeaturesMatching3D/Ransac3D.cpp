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
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process", "[process]" ) 
	{
	//Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInputCache = 
	//	new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	//Mocks::PointCloudToPclPointCloudConverter* mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	//ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	//Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	//Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	//ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	//Create a sample sphere
	VisualPointFeatureVector3DPtr sourceSet =  new VisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet =  new VisualPointFeatureVector3D();
	unsigned numberOfPoints = 0;
	for(float alpha = 0; alpha < 2 * M_PI; alpha += 0.1)
		{
		for(float beta = 0; beta < 2*M_PI; beta += 0.1)
			{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			AddPoint(*sourceSet, spherePoint.x, spherePoint.y, spherePoint.z);
			AddPoint(*sinkSet, spherePoint.x, spherePoint.y, spherePoint.z);
			AddDescriptorComponent(*sourceSet, numberOfPoints, numberOfPoints);
			AddDescriptorComponent(*sinkSet, numberOfPoints, numberOfPoints);
			numberOfPoints++;
			}
		}
	//mockInputConverter->AddBehaviour("Convert", "1", (void*) (&sourceCloud) );
	//mockInputConverter->AddBehaviour("Convert", "2", (void*) (&sinkCloud) );

	//VisualPointFeatureVector3DConstPtr featuresVector = new VisualPointFeatureVector3D();
	//mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

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
