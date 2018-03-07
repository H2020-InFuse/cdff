/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixDecomposition.cpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFN EssentialMatrixDecomposition.
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
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Errors/Assert.hpp>
#include <DataGenerators/SyntheticGenerators/CameraPair.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace DataGenerators;

/* --------------------------------------------------------------------------
 *
 * Test Class
 *
 * --------------------------------------------------------------------------
 */
class EssentialMatrixTest 
	{
	public:
		static void RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences = 20);
		static void FailureTest();

	private:
		static const double EPSILON;
		static Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache;
		static Mocks::MatToPose3DConverter* mockEssentialPoseConverter;

		static void SetupMocksAndStubs();
		static MatrixWrapper::Matrix3dPtr Convert(cv::Mat cvFundamentalMatrix);
		static void ValidateOutput(PoseWrapper::Transform3DConstPtr output, PoseWrapper::Pose3DConstPtr secondCameraPose);
	};

void EssentialMatrixTest::RandomCorrespondencesTest(PoseWrapper::Pose3DConstPtr secondCameraPose, unsigned numberOfCorrespondences)
	{
	SetupMocksAndStubs();

	CameraPair cameraPair;
	cameraPair.SetSecondCameraPose(secondCameraPose);

	cv::Mat cvFundamentalMatrix = cameraPair.GetFundamentalMatrix();
	CorrespondenceMap2DConstPtr input = cameraPair.GetSomeRandomCorrespondences(numberOfCorrespondences);	

	Matrix3dPtr fundamentalMatrix = Convert(cvFundamentalMatrix);

	EssentialMatrixDecomposition essential;
	essential.correspondenceMapInput(input);
	essential.fundamentalMatrixInput(fundamentalMatrix);
	essential.process();

	Transform3DConstPtr output = essential.transformOutput();
	bool success = essential.successOutput();
	REQUIRE(success == true);
	ValidateOutput(output, secondCameraPose);

	delete(input);
	delete(fundamentalMatrix);
	delete(output);
	}

void EssentialMatrixTest::FailureTest()
	{
	SetupMocksAndStubs();

	CorrespondenceMap2DPtr input = new CorrespondenceMap2D();
	for(unsigned index = 0; index < 15; index++)
		{		
		Point2D source = { .x = 0 , .y= 0};
		Point2D sink = { .x = 0 , .y= 0};
		AddCorrespondence(*input, source, sink, 1);
		}

	Matrix3dPtr fundamentalMatrix = NewMatrix3d(IDENTITY);

	EssentialMatrixDecomposition essential;
	essential.correspondenceMapInput(input);
	essential.fundamentalMatrixInput(fundamentalMatrix);
	essential.process();

	Transform3DConstPtr output = essential.transformOutput();
	bool success = essential.successOutput();
	
	REQUIRE(success == false);
	
	delete(input);
	delete(output);
	}

const double EssentialMatrixTest::EPSILON = 0.0001;
Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* EssentialMatrixTest::stubEssentialPoseCache = NULL;
Mocks::MatToPose3DConverter* EssentialMatrixTest::mockEssentialPoseConverter = NULL;

void EssentialMatrixTest::SetupMocksAndStubs()
	{
	static bool setupDone = false;
	if (setupDone)
		{
		return;
		}
	setupDone = true;

	stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);	
	}

MatrixWrapper::Matrix3dPtr EssentialMatrixTest::Convert(cv::Mat cvFundamentalMatrix)
	{
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
	return fundamentalMatrix;
	}

#define REQUIRE_CLOSE(a, b) \
	REQUIRE( a <= b + EPSILON); \
	REQUIRE( a >= b - EPSILON); \

void EssentialMatrixTest::ValidateOutput(PoseWrapper::Transform3DConstPtr output, PoseWrapper::Pose3DConstPtr secondCameraPose)
	{
	REQUIRE_CLOSE( GetXPosition(*output), GetXPosition(*secondCameraPose));
	REQUIRE_CLOSE( GetYPosition(*output), GetYPosition(*secondCameraPose));
	REQUIRE_CLOSE( GetZPosition(*output), GetZPosition(*secondCameraPose));
	REQUIRE_CLOSE( GetXOrientation(*output), GetXOrientation(*secondCameraPose));
	REQUIRE_CLOSE( GetYOrientation(*output), GetYOrientation(*secondCameraPose));
	REQUIRE_CLOSE( GetZOrientation(*output), GetZOrientation(*secondCameraPose));
	REQUIRE_CLOSE( GetWOrientation(*output), GetWOrientation(*secondCameraPose));
	}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Success Call to process for Translation", "[processTranslation]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 0, 1);

	EssentialMatrixTest::RandomCorrespondencesTest(secondCameraPose);
	}

TEST_CASE( "Success Call to process for Rototranslation", "[processRototranslation]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 1, 0, 0);
	SetOrientation(*secondCameraPose, 0, 0, 1, 0);

	EssentialMatrixTest::RandomCorrespondencesTest(secondCameraPose);
	}

TEST_CASE( "Success Call to process for RotoTranslation 2", "[processRototranslation2]" ) 
	{
	Pose3DPtr secondCameraPose = new Pose3D();
	SetPosition(*secondCameraPose, 0, 1, 0);
	SetOrientation(*secondCameraPose, 0, 0, std::sin(M_PI/4), std::sin(M_PI/4));

	EssentialMatrixTest::RandomCorrespondencesTest(secondCameraPose);
	}

TEST_CASE( "Fail Call to process", "[processFail]" ) 
	{
	EssentialMatrixTest::FailureTest();
	}

TEST_CASE( "Call to configure", "[configure]" )
	{
	EssentialMatrixDecomposition essential;
	essential.setConfigurationFile("../tests/ConfigurationFiles/DFNs/CamerasTransformEstimation/EssentialMatrixDecomposition_Conf1.yaml");
	essential.configure();	
	}

/** @} */
