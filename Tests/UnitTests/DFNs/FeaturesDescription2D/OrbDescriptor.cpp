/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDescriptor.cpp
 * @date 21/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN OrbDescriptor.
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
#include <FeaturesDescription2D/OrbDescriptor.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process", "[process]" ) 
	{
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	Mocks::MatToVisualPointFeatureVector2DConverter* mockOutputConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubOutputCache, mockOutputConverter);

	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));	
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	VisualPointFeatureVector2DConstPtr featuresVector = new VisualPointFeatureVector2D();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&featuresVector) );

	FramePtr inputFrame = new Frame();
	VisualPointFeatureVector2DPtr inputFeaturesSet = new VisualPointFeatureVector2D();

	OrbDescriptor orb;
	orb.imageInput(inputFrame);
	orb.featuresSetInput(inputFeaturesSet);
	orb.process();

	VisualPointFeatureVector2DConstPtr output = orb.featuresSetWithDescriptorsOutput();
	
	REQUIRE(GetNumberOfPoints(*output) == GetNumberOfPoints(*featuresVector));
	delete(output);
	delete(inputFrame);
	delete(inputFeaturesSet);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	OrbDescriptor orb;
	orb.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription2D/OrbDescriptor_Conf1.yaml");
	orb.configure();	
	}

/** @} */
