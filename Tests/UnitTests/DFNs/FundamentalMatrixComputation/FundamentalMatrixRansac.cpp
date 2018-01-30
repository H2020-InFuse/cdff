/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixRansac.cpp
 * @date 26/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN FundamentalMatrixRansac.
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
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace MatrixWrapper;
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

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < NUMBER_OF_CORRESPONDENCES; index++)
		{		
		double indexDouble = (double)index;
		Point2D sink;
		sink.x = std::sqrt(indexDouble)*10;
		sink.y = indexDouble;

		double cx = sink.x * F11 + sink.y * F21 + F31;
		double cy = sink.x * F12 + sink.y * F22 + F32;
		double c = sink.x * F13 + sink.y * F23 + F33;

		Point2D source;
		source.y = indexDouble;
		source.x = -(c + (source.y)*cy)/cx;

		AddCorrespondence(*input, source, sink, 1);
		}

	FundamentalMatrixRansac ransac;
	ransac.correspondenceMapInput(input);
	ransac.process();

	Matrix3dConstPtr output = ransac.fundamentalMatrixOutput();
	bool success = ransac.successOutput();
	
	REQUIRE(success == true);

	unsigned inliersCount = 0;
	for(unsigned index = 0; index < NUMBER_OF_CORRESPONDENCES; index++)
		{
		Point2D source = GetSource(*input, index);
		Point2D sink = GetSink(*input, index);
		
		double cx = sink.x * GetElement(*output, 0, 0) + sink.y * GetElement(*output, 1, 0) + GetElement(*output, 2, 0);
		double cy = sink.x * GetElement(*output, 0, 1) + sink.y * GetElement(*output, 1, 1) + GetElement(*output, 2, 1);
		double c = sink.x * GetElement(*output, 0, 2) + sink.y * GetElement(*output, 1, 2) + GetElement(*output, 2, 2);

		double total = cx * source.x + cy * source.y + c; 
	
		REQUIRE( total < EPSILON);
		REQUIRE( total > -EPSILON);				
		}
	
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

	FundamentalMatrixRansac ransac;
	ransac.correspondenceMapInput(input);
	ransac.process();

	Matrix3dConstPtr output = ransac.fundamentalMatrixOutput();
	bool success = ransac.successOutput();
	
	REQUIRE(success == false);
	
	delete(input);
	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	FundamentalMatrixRansac ransac;
	ransac.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac_Conf1.yaml");
	ransac.configure();	
	}

/** @} */
