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
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <FeaturesDescription2D/OrbDescriptor.hpp>
#include <Converters/MatToFrameConverter.hpp>

using namespace CDFF::DFN::FeaturesDescription2D;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (ORB descriptor)", "[process]" ) 
{
	// Prepare input data
	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
	FrameConstPtr inputFrame = MatToFrameConverter().Convert(inputImage);
	VisualPointFeatureVector2DConstPtr inputFeatures = NewVisualPointFeatureVector2D();

	// Instantiate DFN
	OrbDescriptor* orb = new OrbDescriptor;

	// Send input data to DFN
	orb->frameInput(*inputFrame);
	orb->featuresInput(*inputFeatures);

	// Run DFN
	orb->process();

	// Query output data from DFN
	const VisualPointFeatureVector2D& output = orb->featuresOutput();

	// Cleanup
	delete inputFeatures;
	delete orb;
}

TEST_CASE( "Call to configure (ORB descriptor)", "[configure]" )
{
	// Instantiate DFN
	OrbDescriptor* orb = new OrbDescriptor;

	// Setup DFN
	orb->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesDescription2D/OrbDescriptor_Conf1.yaml");
	orb->configure();

	// Cleanup
	delete orb;
}

/** @} */
