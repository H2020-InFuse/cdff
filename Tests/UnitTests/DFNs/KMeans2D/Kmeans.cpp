// includes
#include <catch.hpp>
#include <KMeans2D/Kmeans.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <iostream>

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

	// ----- build I/O converters
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, Frame >* stubOutputCache = new Stubs::CacheHandler<cv::Mat, Frame>();
	Mocks::MatToFrameConverter* mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, Frame, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);

	// ----- build inputs
	cv::Mat inputImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));	
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputImage) );

	// ----- build outputs
	FrameConstPtr CannyEdge = new Frame();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&CannyEdge) );

	// ----- processing
	Kmeans kmeans;
	kmeans.imageInput(new Frame());
	kmeans.process();

	// ----- save outputs
	FrameConstPtr output = kmeans.KMeansImageOutput();

	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	Kmeans kmeans;
	kmeans.setConfigurationFile("../tests/ConfigurationFiles/DFNs/KMeans2D/Kmeans_Conf1.yaml");
	kmeans.configure();	
	}

/** @} */