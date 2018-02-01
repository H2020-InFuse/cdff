/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixRansac.cpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN EssentialMatrixRansac.
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
#include <CamerasTransformEstimation/EssentialMatrixRansac.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process", "[processSuccess]" ) 
	{
	const double EPSILON = 0.001;
	const unsigned NUMBER_OF_CORRESPONDENCES = 15;
	//These are fundamental matrix elements:
	const double F11 = -0.00310695;
	const double F12 = -0.0025646 ;
	const double F13 = 2.96584;
	const double F21 = -0.028094;
	const double F22 = -0.00771621;
	const double F23 = 56.3813;
	const double F31 = 13.1905 ;
	const double F32 = -29.2007;
	const double F33 = -9999.79;

	double fx = 1;
	double fy = 1;
	double cx = 0;
	double cy = 0;

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < NUMBER_OF_CORRESPONDENCES; index++)
		{		
		double indexDouble = (double)index;
		Point2D sink, normalizedSink;
		sink.x = std::sqrt(indexDouble)*10;
		sink.y = indexDouble;

		normalizedSink.x = fx * sink.x + cx;
		normalizedSink.y = fy * sink.y + cy;

		double cx = sink.x * F11 + sink.y * F21 + F31;
		double cy = sink.x * F12 + sink.y * F22 + F32;
		double c = sink.x * F13 + sink.y * F23 + F33;

		Point2D source, normalizedSource;
		normalizedSource.y = indexDouble;
		normalizedSource.x = -(c + (normalizedSink.y)*cy)/cx;

		source.y = (normalizedSource.y -cy)/fy;
		source.x = (normalizedSource.x -cx)/fx;

		AddCorrespondence(*input, source, sink, 1);
		}

	EssentialMatrixRansac ransac;
	ransac.correspondenceMapInput(input);
	ransac.process();

	Transform3DConstPtr output = ransac.transformOutput();
	bool success = ransac.successOutput();
	
	REQUIRE(success == true);
	
	delete(input);
	delete(output);
	}

TEST_CASE( "Fail Call to process", "[processFail]" ) 
	{
	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < 15; index++)
		{		
		Point2D source = { .x = 0 , .y= 0};
		Point2D sink = { .x = 0 , .y= 0};
		AddCorrespondence(*input, source, sink, 1);
		}

	EssentialMatrixRansac ransac;
	ransac.correspondenceMapInput(input);
	ransac.process();

	Transform3DConstPtr output = ransac.transformOutput();
	bool success = ransac.successOutput();
	
	REQUIRE(success == false);
	
	delete(input);
	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	EssentialMatrixRansac ransac;
	ransac.setConfigurationFile("../tests/ConfigurationFiles/DFNs/CamerasTransformEstimation/EssentialMatrixRansac_Conf1.yaml");
	ransac.configure();	
	}

/** @} */
