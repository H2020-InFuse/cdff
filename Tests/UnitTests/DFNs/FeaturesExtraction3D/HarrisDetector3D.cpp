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
 * Unit Test for the DFN HarrisDetector3D.
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
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process", "[process]" ) 
	{
	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInputCache = 
		new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	Mocks::PointCloudToPclPointCloudConverter* mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	//Create a sample sphere
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud =  boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
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

	HarrisDetector3D harris;
	harris.pointCloudInput(new PointCloud());
	harris.process();

	VisualPointFeatureVector3DConstPtr output = harris.featuresSetOutput();

	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	HarrisDetector3D harris;
	harris.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction3D/HarrisDetector3D_Conf1.yaml");
	harris.configure();	
	}

/** @} */
