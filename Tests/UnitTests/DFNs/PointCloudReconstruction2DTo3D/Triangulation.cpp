/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN Triangulation.
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
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
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

	
	Matrix3dConstPtr fundamentalMatrix = NewMatrix3d(IDENTITY);
	Point2DConstPtr secondEpipole = new Point2D();

	Triangulation triangulation;
	triangulation.correspondenceMapInput(input);
	triangulation.fundamentalMatrixInput(fundamentalMatrix);
	triangulation.secondEpipoleInput(secondEpipole);
	triangulation.process();

	PointCloudConstPtr output = triangulation.pointCloudOutput();
	REQUIRE ( GetNumberOfPoints(*output) == GetNumberOfCorrespondences(*input) );
	
	delete(input);
	delete(fundamentalMatrix);
	delete(secondEpipole);
	delete(output);
	}

/** @} */
