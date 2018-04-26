/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortion.cpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN ImageUndistortion.
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
#include <ImageFiltering/ImageUndistortion.hpp>
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
TEST_CASE( "Call to process (image undistortion)", "[process]" ) 
	{
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	Mocks::MatToFrameConverter* mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);

	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));	
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	FrameConstPtr outputImage = new Frame();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&outputImage) );

	ImageUndistortion undistortion;
	FrameConstPtr input = new Frame();
	undistortion.imageInput(input);
	undistortion.process();

	FrameConstPtr output = undistortion.filteredImageOutput();
	
	delete(input);
	delete(output);
	}

TEST_CASE( "Call to configure (image undistortion)", "[configure]" )
	{
	ImageUndistortion undistortion;
	undistortion.setConfigurationFile("../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortion_Conf1.yaml");
	undistortion.configure();	
	}

/** @} */
