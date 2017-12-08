/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector2D.cpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN HarrisDetector2D.
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
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Types;
using namespace Common;
using namespace Converters;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process", "[process]" ) 
	{
	Stubs::CacheHandler<CppTypes::Frame::ConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<CppTypes::Frame::ConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<CppTypes::Frame::ConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2D* >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2D*>();
	Mocks::MatToVisualPointFeatureVector2DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2D*, MatToVisualPointFeatureVector2DConverter>::Instance(stubOutputCache, mockOutputConverter);

	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));	
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	VisualPointFeatureVector2D* featuresVector = new VisualPointFeatureVector2D();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

	HarrisDetector2D harris;
	harris.process();

	const VisualPointFeatureVector2D* output = harris.featuresSetOutput();

	REQUIRE(output->list.size == featuresVector->list.size);
	//No need to delete StubCacheHandler, MockImageTypeToMatConverter, StubCacheHandler, MockMatToVisualPointFeatureVector2DConverter
	//ConversionCache destructor takes care of that.
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	HarrisDetector2D harris;
	harris.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction2D/HarrisDetector2D_Conf1.yaml");
	harris.configure();	
	}

/** @} */
