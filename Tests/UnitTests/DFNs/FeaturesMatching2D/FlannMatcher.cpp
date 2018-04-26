/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FlannMatcher.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN FlannMatcher.
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
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (FLANN registration)", "[process]" ) 
	{
	Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat >* stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	Mocks::VisualPointFeatureVector2DToMatConverter* mockInputConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubInputCache, mockInputConverter);

	const unsigned NUMBER_OF_POINTS = 10;
	const unsigned DESCRIPTOR_LENGTH = 8;
	cv::Mat inputImage(NUMBER_OF_POINTS, DESCRIPTOR_LENGTH+2, CV_32FC1, cv::Scalar(0));	
	for(unsigned pointIndex = 0; pointIndex < NUMBER_OF_POINTS; pointIndex++)
		{
		inputImage.at<float>(pointIndex, 0) = 10;
		inputImage.at<float>(pointIndex, 1) = (float)pointIndex + 10;
		for(unsigned componentIndex = 0; componentIndex < DESCRIPTOR_LENGTH; componentIndex++)
			{
			inputImage.at<float>(pointIndex, componentIndex+2) = (float)pointIndex + (float)componentIndex / 100; 
			}
		}
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );
	mockInputConverter->AddBehaviour("Convert", "2", (void*) (&inputImage) );

	FlannMatcher flann;
	flann.sourceFeaturesVectorInput(new VisualPointFeatureVector2D());
	flann.sinkFeaturesVectorInput(new VisualPointFeatureVector2D());
	flann.process();

	CorrespondenceMap2DConstPtr output = flann.correspondenceMapOutput();
	
	delete(output);
	}

TEST_CASE( "Call to configure (FLANN registration)", "[configure]" )
	{
	FlannMatcher flann;
	flann.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching2D/FlannMatcher_Conf1.yaml");
	flann.configure();	
	}

/** @} */
