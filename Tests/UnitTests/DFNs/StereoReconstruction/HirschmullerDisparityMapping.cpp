/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN Implementation: HirschmullerDisparityMapping.
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
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Errors/Assert.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process (Hirschmueller disparity mapping)", "[process]" ) 
	{
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	cv::Mat cvImage = cv::imread("../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cv::Mat leftImage = cvImage( cv::Rect(0, 0, cvImage.cols/2, cvImage.rows) );
	cv::Mat rightImage = cvImage( cv::Rect(cvImage.cols/2, 0, cvImage.cols/2, cvImage.rows) );
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&leftImage) );
	mockInputConverter->AddBehaviour("Convert", "2", (void*) (&rightImage) );

	HirschmullerDisparityMapping disparityMapping;
	disparityMapping.leftImageInput(new Frame());
	disparityMapping.rightImageInput(new Frame());
	disparityMapping.process();

	PointCloudConstPtr outputCloud = disparityMapping.pointCloudOutput();
	delete(outputCloud);
	}

TEST_CASE( "Success Call to configure (Hirschmueller disparity mapping)", "[configure]" ) 
	{
	HirschmullerDisparityMapping disparityMapping;
	disparityMapping.setConfigurationFile("../tests/ConfigurationFiles/DFNs/StereoReconstruction/HirschmullerDisparityMapping_Conf1.yaml");
	disparityMapping.configure();	
	}


/** @} */
