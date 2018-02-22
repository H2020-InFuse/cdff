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
#include <CamerasTransformEstimation/EssentialMatrixComputation.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process", "[processSuccess]" ) 
	{
	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	const double EPSILON = 0.001;
	const unsigned NUMBER_OF_CORRESPONDENCES = 3;

	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0));
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat secondProjection(3, 4, CV_32FC1, cv::Scalar(0));
	secondProjection.at<float>(0,0) = 1;
	secondProjection.at<float>(1,1) = 1;
	secondProjection.at<float>(2,2) = 1;
	secondProjection.at<float>(0,3) = 1;
	
	cv::Mat skew(3, 3, CV_32FC1, cv::Scalar(0));
	skew.at<float>(1, 2) = -1;
	skew.at<float>(2, 1) = +1;

	cv::Mat cvFundamentalMatrix = skew * secondProjection * identityProjection.t();

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < NUMBER_OF_CORRESPONDENCES; index++)
		{		
		cv::Mat point3d(4, 1, CV_32FC1);
		point3d.at<float>(0,0) = ( (float) index ) / 4;
		point3d.at<float>(1,0) = std::sqrt((float) index);
		if ( index <= NUMBER_OF_CORRESPONDENCES /2)
			{
			point3d.at<float>(2,0) = ((float) index)/8 + 1.1;
			}
		else
			{
			point3d.at<float>(2,0) = 2000*((float)index);			
			}
		point3d.at<float>(3,0) = 1;
		
		cv::Mat firstPoint2d = identityProjection * point3d;
		cv::Mat secondPoint2d = secondProjection * point3d;

		cv::Mat test = firstPoint2d.t() * cvFundamentalMatrix * secondPoint2d;
		ASSERT ( std::abs( test.at<float>(0,0) ) < EPSILON, "Fundamental Matrix is wrong");

		Point2D source, sink;

		source.x = firstPoint2d.at<float>(0, 0) / firstPoint2d.at<float>(2, 0);
		source.y = firstPoint2d.at<float>(1, 0) / firstPoint2d.at<float>(2, 0);
		sink.x = secondPoint2d.at<float>(0, 0) / secondPoint2d.at<float>(2, 0);
		sink.y = secondPoint2d.at<float>(1, 0) / secondPoint2d.at<float>(2, 0);

		AddCorrespondence(*input, source, sink, 1);
		}

	Matrix3dPtr fundamentalMatrix = NewMatrix3d(IDENTITY);
	SetElement(*fundamentalMatrix, 0, 0, cvFundamentalMatrix.at<float>(0,0) );
	SetElement(*fundamentalMatrix, 0, 1, cvFundamentalMatrix.at<float>(0,1) );
	SetElement(*fundamentalMatrix, 0, 2, cvFundamentalMatrix.at<float>(0,2) );
	SetElement(*fundamentalMatrix, 1, 0, cvFundamentalMatrix.at<float>(1,0) );
	SetElement(*fundamentalMatrix, 1, 1, cvFundamentalMatrix.at<float>(1,1) );
	SetElement(*fundamentalMatrix, 1, 2, cvFundamentalMatrix.at<float>(1,2) );
	SetElement(*fundamentalMatrix, 2, 0, cvFundamentalMatrix.at<float>(2,0) );
	SetElement(*fundamentalMatrix, 2, 1, cvFundamentalMatrix.at<float>(2,1) );
	SetElement(*fundamentalMatrix, 2, 2, cvFundamentalMatrix.at<float>(2,2) );

	EssentialMatrixComputation essential;
	essential.correspondenceMapInput(input);
	essential.fundamentalMatrixInput(fundamentalMatrix);
	essential.process();

	Transform3DConstPtr output = essential.transformOutput();
	bool success = essential.successOutput();
	
	REQUIRE(success == true);
	REQUIRE( GetXPosition(*output) == 1);
	REQUIRE( GetYPosition(*output) == 0);
	REQUIRE( GetZPosition(*output) == 0);
	
	delete(input);
	delete(fundamentalMatrix);
	delete(output);
	}

TEST_CASE( "Fail Call to process", "[processFail]" ) 
	{
	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < 15; index++)
		{		
		Point2D source = { .x = 0 , .y= 0};
		Point2D sink = { .x = 0 , .y= 0};
		AddCorrespondence(*input, source, sink, 1);
		}

	Matrix3dPtr fundamentalMatrix = NewMatrix3d(IDENTITY);

	EssentialMatrixComputation essential;
	essential.correspondenceMapInput(input);
	essential.fundamentalMatrixInput(fundamentalMatrix);
	essential.process();

	Transform3DConstPtr output = essential.transformOutput();
	bool success = essential.successOutput();
	
	REQUIRE(success == false);
	
	delete(input);
	delete(output);
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	EssentialMatrixComputation essential;
	essential.setConfigurationFile("../tests/ConfigurationFiles/DFNs/CamerasTransformEstimation/EssentialMatrixComputation_Conf1.yaml");
	essential.configure();	
	}

/** @} */
