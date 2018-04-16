/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CannyDetector.cpp
 * @date 16/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN SobelDerivative.
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
#include <SobelDerivative/SobelScharr.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;

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

	Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	Mocks::MatToFrameConverter* mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);

	cv::Mat inputImage;
	cv::Mat testImage = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
 	cvtColor(testImage, inputImage, COLOR_BGR2GRAY );
		
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	FrameConstPtr outputImage = new Frame();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&outputImage) );

	
	SobelScharr sobelgrad;
	
	FrameConstPtr input = new Frame();
	sobelgrad.imageInput(input);
	sobelgrad.process();

	FrameConstPtr outputx = sobelgrad.sobelGradxOutput();
	FrameConstPtr outputy = sobelgrad.sobelGradyOutput();
	
	delete(input);
	delete(outputx);
	delete(outputy);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	SobelScharr sobelgrad;
	sobelgrad.setConfigurationFile("../tests/ConfigurationFiles/DFNs/SobelDerivative/SobelScharr_Conf.yaml");
	sobelgrad.configure();	
	}

/** @} */
