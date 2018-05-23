// includes
#include <catch.hpp>
#include <NormalMap2D/NormalsExtractor2D.hpp>
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
	cv::Mat depthImage(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));	
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&depthImage) );

	// ----- build outputs
	FrameConstPtr Normals = new Frame();
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&Normals) );

	// ----- processing
	NormalsExtractor2D normals;
	normals.imageInput(new Frame());
	normals.process();

	// ----- save outputs
	FrameConstPtr output = normals.NormalMapOutput();

	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	NormalsExtractor2D normals;
	normals.setConfigurationFile("../tests/ConfigurationFiles/DFNs/NormalMap2D/NormalsExtractor2D_Conf1.yaml");
	normals.configure();	
	}

/** @} */