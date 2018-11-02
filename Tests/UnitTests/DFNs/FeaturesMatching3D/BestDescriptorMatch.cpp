/**
 * @author Alessandro Bianco
 */

/**
 * Unit tests for the DFN BestDescriptorMatch
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesMatching3D/BestDescriptorMatch.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <Pose.hpp>

#include <boost/make_shared.hpp>

using namespace CDFF::DFN::FeaturesMatching3D;
using namespace CDFF::DFN::FeaturesDescription3D;
using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace SupportTypes;

TEST_CASE( "DFN processing step succeeds (Best Descriptor Match)", "[process]" )
{
	// Prepare simpler input data
	VisualPointFeatureVector3DPtr sourceSet = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet = NewVisualPointFeatureVector3D();

	// Instantiate DFN
	BestDescriptorMatch *bestDescriptorMatch = new BestDescriptorMatch;

	// Send input data to DFN
	bestDescriptorMatch->sourceFeaturesInput(*sourceSet);
	bestDescriptorMatch->sinkFeaturesInput(*sinkSet);

	// Run DFN
	bestDescriptorMatch->process();

	// Query output data from DFN
	const Pose3D& output = bestDescriptorMatch->transformOutput();

	REQUIRE(GetXPosition(output) == 0);
	REQUIRE(GetYPosition(output) == 0);
	REQUIRE(GetZPosition(output) == 0);
	REQUIRE(GetXOrientation(output) == 0);
	REQUIRE(GetYOrientation(output) == 0);
	REQUIRE(GetZOrientation(output) == 0);
	REQUIRE(GetWOrientation(output) == 0);

	// Cleanup
	delete bestDescriptorMatch;
	delete sourceSet;
	delete sinkSet;
}

TEST_CASE( "DFN configuration succeeds (Best Descriptor Match)", "[configure]" )
{
	// Instantiate DFN
	BestDescriptorMatch *bestDescriptorMatch = new BestDescriptorMatch;

	// Setup DFN
	bestDescriptorMatch->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/BestDescriptorMatch_Conf1.yaml");
	bestDescriptorMatch->configure();

	// Cleanup
	delete bestDescriptorMatch;
}

#define REQUIRE_CLOSE(a, b) REQUIRE(a < b + 0.001); REQUIRE(a > b - 0.001);

TEST_CASE( "DFN processing step succeeds with data (Best Descriptor Match)", "[configure]" )
{
	// Prepare simpler input data
	VisualPointFeatureVector3DPtr sourceSet = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet = NewVisualPointFeatureVector3D();

	const float INCREMENT = 0.1;
	int pointIndex = 0;
	for(float x = 0; x <= 1; x += INCREMENT)
		{
		AddPoint(*sourceSet, x, 2*x, std::sqrt(x) );
		AddPoint(*sinkSet, x+1, 2*x+1, std::sqrt(x) );
		for(int i = 0; i < 10; i++)
			{
			AddDescriptorComponent(*sourceSet, pointIndex, x*2);
			AddDescriptorComponent(*sinkSet, pointIndex, x*2);
			}
		pointIndex++;		
		}	

	// Instantiate DFN
	BestDescriptorMatch *bestDescriptorMatch = new BestDescriptorMatch;

	// Setup DFN
	bestDescriptorMatch->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/BestDescriptorMatch_Conf2.yaml");
	bestDescriptorMatch->configure();

	// Send input data to DFN
	bestDescriptorMatch->sourceFeaturesInput(*sourceSet);
	bestDescriptorMatch->sinkFeaturesInput(*sinkSet );

	// Run DFN
	bestDescriptorMatch->process();

	// Query output data from DFN
	const Pose3D& output = bestDescriptorMatch->transformOutput();
	bool success = bestDescriptorMatch->successOutput();

	PRINT_TO_LOG("pose", ToString(output) );
	REQUIRE(success);
	REQUIRE_CLOSE(GetXPosition(output), 1);
	REQUIRE_CLOSE(GetYPosition(output), 1);
	REQUIRE_CLOSE(GetZPosition(output), 0);
	REQUIRE_CLOSE(GetXOrientation(output), 0);
	REQUIRE_CLOSE(GetYOrientation(output), 0);
	REQUIRE_CLOSE(GetZOrientation(output), 0);
	REQUIRE_CLOSE(GetWOrientation(output), 1);

	// Cleanup
	delete bestDescriptorMatch;
	delete sourceSet;
	delete sinkSet;
}

#define REQUIRE_LESS_CLOSE(a, b) REQUIRE(a < b + 0.3); REQUIRE(a > b - 0.3);

TEST_CASE( "DFN processing step succeeds with data with a bad point (Best Descriptor Match)", "[configure]" )
{
	// Prepare simpler input data
	VisualPointFeatureVector3DPtr sourceSet = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr sinkSet = NewVisualPointFeatureVector3D();

	const float INCREMENT = 0.1;
	int pointIndex = 0;
	for(float x = 0; x <= 1; x += INCREMENT)
		{
		AddPoint(*sourceSet, x, 2*x, std::sqrt(x) );
		AddPoint(*sinkSet, x+1, 2*x+1, std::sqrt(x) );
		for(int i = 0; i < 10; i++)
			{
			AddDescriptorComponent(*sourceSet, pointIndex, x*2);
			AddDescriptorComponent(*sinkSet, pointIndex, x*2);
			}
		pointIndex++;		
		}
	AddPoint(*sourceSet, 1.3, 2.6, std::sqrt(1.3) );
	AddPoint(*sinkSet, 4, 4, std::sqrt(1.3) );
	for(int i = 0; i < 10; i++)
		{
		AddDescriptorComponent(*sourceSet, pointIndex, 2);
		AddDescriptorComponent(*sinkSet, pointIndex, 2);
		}	

	// Instantiate DFN
	BestDescriptorMatch *bestDescriptorMatch = new BestDescriptorMatch;

	// Setup DFN
	bestDescriptorMatch->setConfigurationFile("../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/BestDescriptorMatch_Conf2.yaml");
	bestDescriptorMatch->configure();

	// Send input data to DFN
	bestDescriptorMatch->sourceFeaturesInput(*sourceSet);
	bestDescriptorMatch->sinkFeaturesInput(*sinkSet );

	// Run DFN
	bestDescriptorMatch->process();

	// Query output data from DFN
	const Pose3D& output = bestDescriptorMatch->transformOutput();
	bool success = bestDescriptorMatch->successOutput();

	PRINT_TO_LOG("pose", ToString(output) );
	REQUIRE(success);
	REQUIRE_LESS_CLOSE(GetXPosition(output), 1);
	REQUIRE_LESS_CLOSE(GetYPosition(output), 1);
	REQUIRE_LESS_CLOSE(GetZPosition(output), 0);
	REQUIRE_LESS_CLOSE(GetXOrientation(output), 0);
	REQUIRE_LESS_CLOSE(GetYOrientation(output), 0);
	REQUIRE_LESS_CLOSE(GetZOrientation(output), 0);
	REQUIRE_LESS_CLOSE(GetWOrientation(output), 1);

	// Cleanup
	delete bestDescriptorMatch;
	delete sourceSet;
	delete sinkSet;
}

/** @} */
