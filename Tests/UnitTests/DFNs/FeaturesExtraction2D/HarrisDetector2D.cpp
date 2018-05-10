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
 * Unit Test for the DFN HarrisDetector2D.
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
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <MatToFrameConverter.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (2D Harris detector)", "[process]" ) 
{
	// Prepare input data
	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
	MatToFrameConverter matToFrame;
	FrameConstPtr inputFrame = matToFrame.Convert(inputImage);

	// Instantiate DFN
	HarrisDetector2D* harris = new HarrisDetector2D;

	// Send input data to DFN
	harris->frameInput(*inputFrame);

	// Run DFN
	harris->process();

	// Query output data from DFN
	const VisualPointFeatureVector2D& output = harris->featuresOutput();
}

TEST_CASE( "Call to configure (2D Harris detector)", "[configure]" )
{
	// Instantiate DFN
	HarrisDetector2D* harris = new HarrisDetector2D;

	// Setup DFN
	harris->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction2D/HarrisDetector2D_Conf1.yaml");
	harris->configure();
}

/** @} */
