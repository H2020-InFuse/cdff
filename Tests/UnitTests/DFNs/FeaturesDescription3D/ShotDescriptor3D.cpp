/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ShotDescriptor3D.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN ShotDescriptor3D.
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
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
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

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>* stubInputNormalsCache = 
		new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	//Create a sample sphere
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud =  boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(float alpha = 0; alpha < 2 * M_PI; alpha += 0.01)
		{
		for(float beta = 0; beta < 2*M_PI; beta += 0.01)
			{
			pcl::PointXYZ spherePoint;
			spherePoint.x = std::cos(alpha) * std::cos(beta);
			spherePoint.y = std::sin(alpha);
			spherePoint.z = std::sin(beta);
			inputCloud->points.push_back(spherePoint);
			}
		}
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputCloud) );

	pcl::PointCloud<pcl::Normal>::Ptr inputNormalsCloud =  boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	mockInputNormalsConverter->AddBehaviour("Convert", "1", (void*) (&inputNormalsCloud) );

	VisualPointFeatureVector3DPtr inputFeaturesVector = NewVisualPointFeatureVector3D();
	ClearPoints(*inputFeaturesVector);
	AddPoint(*inputFeaturesVector, 0);

	VisualPointFeatureVector3DConstPtr featuresVector = NewVisualPointFeatureVector3D();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

	ShotDescriptor3D shot;
	shot.pointCloudInput(new PointCloud());
	shot.featuresSetInput(inputFeaturesVector);
	shot.process();

	VisualPointFeatureVector3DConstPtr output = shot.featuresSetWithDescriptorsOutput();

	REQUIRE(GetNumberOfPoints(*output) == 1);
	delete(output);
	delete(inputFeaturesVector);
	delete(featuresVector);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	ShotDescriptor3D shot;
	shot.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/ShotDescriptor3D_Conf1.yaml");
	shot.configure();	
	}

TEST_CASE( "Fail Configuration due to force but not enabled normal estimation", "[ForceEnanbleContradiction]")
	{
	ShotDescriptor3D shot;
	shot.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/ShotDescriptor3D_ForceEnanbleContradiction.yaml");
	REQUIRE_THROWS( shot.configure() );
	}

TEST_CASE( "Fail Configuration due to enabled normal estimation with no parameters", "[NoEnabledEstimationParameters]")
	{
	ShotDescriptor3D shot;
	shot.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/ShotDescriptor3D_NoEnabledEstimationParameters.yaml");
	REQUIRE_THROWS( shot.configure() );
	}

TEST_CASE( "Fail Configuration due to enabled normal estimation with conflicting parameters", "[ConflictingEnabledEstimationParameters]")
	{
	ShotDescriptor3D shot;
	shot.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/ShotDescriptor3D_ConflictingEnabledEstimationParameters.yaml");
	REQUIRE_THROWS( shot.configure() );
	}

TEST_CASE( "Failed processing for invalid normal cloud and disabled estimation", "[DisabledEstimationInvalidNormalCloud]")
	{
	ShotDescriptor3D shot;
	shot.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription3D/ShotDescriptor3D_DisabledEstimationInvalidNormalCloud.yaml");
	shot.configure();

	PointCloudPtr inputCloud = new PointCloud();
	ClearPoints(*inputCloud);
	AddPoint(*inputCloud, 0, 0, 0);
	shot.pointCloudInput(inputCloud);

	VisualPointFeatureVector3DPtr featuresSet = new VisualPointFeatureVector3D();
	shot.featuresSetInput(featuresSet);

	PointCloudPtr normalsCloud = new PointCloud();	
	ClearPoints(*normalsCloud);
	shot.normalsCloudInput(normalsCloud);

	REQUIRE_THROWS( shot.process() );
	}

/** @} */
