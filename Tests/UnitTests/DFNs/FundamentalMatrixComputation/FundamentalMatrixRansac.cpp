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
#include <time.h>
#include <DataGenerators/SyntheticGenerators/CameraPair.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace DataGenerators;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Class
 *
 * --------------------------------------------------------------------------
 */
class FundamentalMatrixTest 
	{
	public:
		static void RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences = 20);
		static void FailureTest();

	private:
		static const double EPSILON;

		static void ValidateOutput(MatrixWrapper::Matrix3dConstPtr output, cv::Mat cvFundamentalMatrix);
	};

void FundamentalMatrixTest::RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences)
	{
	CameraPair cameraPair;
	cameraPair.SetSecondCameraPose(secondCameraPose);

	cv::Mat cvFundamentalMatrix = cameraPair.GetFundamentalMatrix();
	CorrespondenceMap2DConstPtr input = cameraPair.GetSomeRandomCorrespondences(numberOfCorrespondences);

	FundamentalMatrixRansac ransac;
	ransac.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac_Conf2.yaml");
	ransac.configure();

	ransac.correspondenceMapInput(input);
	ransac.process();

	Matrix3dConstPtr output = ransac.fundamentalMatrixOutput();
	bool success = ransac.successOutput();	
	REQUIRE(success == true);

	ValidateOutput(output, cvFundamentalMatrix);
	}

void FundamentalMatrixTest::FailureTest()
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

const double FundamentalMatrixTest::EPSILON = 0.0001;

#define REQUIRE_CLOSE(a, b) \
	REQUIRE( a <= b + EPSILON); \
	REQUIRE( a >= b - EPSILON); \

void FundamentalMatrixTest::ValidateOutput(Matrix3dConstPtr output, cv::Mat cvFundamentalMatrix)
	{
	bool commonFactorFound = false;
	float commonFactor = 0;
	for(unsigned row = 0; row < 3; row++)
		{
		for (unsigned column = 0; column < 3; column++)
			{
			bool resultCloseToZero = GetElement(*output, row, column) > -EPSILON && GetElement(*output, row, column) < -EPSILON;
			if (resultCloseToZero)
				{
				bool fundamentalCloseToZero = cvFundamentalMatrix.at<float>(row, column) > -EPSILON && cvFundamentalMatrix.at<float>(row, column) < EPSILON;
				REQUIRE(fundamentalCloseToZero == true);
				}
			else
				{
				if (!commonFactorFound)
					{
					commonFactor = cvFundamentalMatrix.at<float>(row, column) / GetElement(*output, row, column);
					}
				else
					{
					float scaledResult = commonFactor * GetElement(*output, row, column);
					REQUIRE_CLOSE(scaledResult, cvFundamentalMatrix.at<float>(row, column));
					}
				}
			}
		}
	}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Fail Call to process", "[processFail]" ) 
	{
	FundamentalMatrixTest::FailureTest();
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	FundamentalMatrixRansac ransac;
	ransac.setConfigurationFile("../tests/ConfigurationFiles/DFNs/FundamentalMatrixComputation/FundamentalMatrixRansac_Conf1.yaml");
	ransac.configure();	
	}

TEST_CASE( "Correct computation on camera translation X", "[fundamentalTranslationX]" )
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 2, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 0, 1);

	FundamentalMatrixTest::RandomCorrespondencesTest(secondCameraPose, 12);
	}

TEST_CASE( "Correct computation on camera translation Y", "[fundamentalTranslationY]" )
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 0, 2, 0);
	SetOrientation(*secondCameraPose, 0, 0, 0, 1);

	FundamentalMatrixTest::RandomCorrespondencesTest(secondCameraPose, 12);
	}

TEST_CASE( "Correct computation on camera translation Z", "[fundamentalTranslationZ]" )
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 0, 0, 2);
	SetOrientation(*secondCameraPose, 0, 0, 0, 1);

	FundamentalMatrixTest::RandomCorrespondencesTest(secondCameraPose, 12);
	}

TEST_CASE( "Correct computation on camera roto-translation", "[fundamentalRototranslation]" )
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 1, 0);
	SetOrientation(*secondCameraPose, 0, 0, 1, 0);

	FundamentalMatrixTest::RandomCorrespondencesTest(secondCameraPose, 12);
	}

TEST_CASE( "Correct computation on camera roto-translation 2", "[fundamentalRototranslation2]" )
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 1, 0);
	SetOrientation(*secondCameraPose, 0, 0, std::sin(M_PI/4), std::sin(M_PI_4));

	FundamentalMatrixTest::RandomCorrespondencesTest(secondCameraPose, 12);
	}

/** @} */
