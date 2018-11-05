/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptor.cpp
 * @date 23/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN OrbDetectorDescriptor.
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
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <Converters/MatToFrameConverter.hpp>

using namespace CDFF::DFN::FeaturesExtraction2D;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ORB)", "[process]" ) 
{
	// Prepare input data
	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
	MatToFrameConverter matToFrame;
	FrameConstPtr inputFrame = matToFrame.Convert(inputImage);

	// Instantiate DFN
	OrbDetectorDescriptor* orb = new OrbDetectorDescriptor;

	// Send input data to DFN
	orb->frameInput(*inputFrame);

	// Run DFN
	orb->process();

	// Query output data from DFN
	const VisualPointFeatureVector2D& output = orb->featuresOutput();
}

TEST_CASE( "Call to configure (ORB)", "[configure]" )
{
	// Instantiate DFN
	OrbDetectorDescriptor* orb = new OrbDetectorDescriptor;

	// Setup DFN
	orb->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesExtraction2D/OrbDetectorDescriptor_Conf1.yaml");
	orb->configure();	
}

/** @} */
