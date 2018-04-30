/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SobelDerivative.cpp
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
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <EdgeDetection/SobelDerivative.hpp>
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

/*

TEST_CASE( "Call to process ( sobel derivatives )", "[process]" ) 
	{
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	Mocks::MatToFrameConverter* mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);

	cv::Mat inputImage;
	cv::Mat testImage = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
 	cvtColor(testImage, inputImage, cv::COLOR_BGR2GRAY );
		
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	FrameConstPtr outputImage = new Frame();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&outputImage) );

	
	SobelDerivative sobelGradient;
	
	FrameConstPtr input = new Frame();
	sobelGradient.imageInput(input);
	sobelGradient.process();

	FrameConstPtr outputx = sobelGradient.sobelGradientXOutput();
	FrameConstPtr outputy = sobelGradient.sobelGradientYOutput();
	
	delete(input);
	delete(outputx);
	delete(outputy);
	}
*/

TEST_CASE( "Call to configure ( sobel derivatives )", "[configure]" )
	{
	SobelDerivative sobelGradient;
	sobelGradient.setConfigurationFile("../tests/ConfigurationFiles/DFNs/SobelDerivative/SobelDerivative_Conf.yaml");
	sobelGradient.configure();	
	}

/** @} */
