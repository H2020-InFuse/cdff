/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixDecompositionTriangulation.cpp
 * @date 15/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This class will test requirement 4.1.1.5 of deliverable 5.5.
 * "Triangulated points are guaranteed by the algorithm to be placed within the field of view of the camera."
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ReconstructionTester.hpp"
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::FundamentalMatrixComputation;
using namespace CDFF::DFN::CamerasTransformEstimation;
using namespace CDFF::DFN::PointCloudReconstruction2DTo3D;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide at six parameters: \n \
(i) the fundamental matrix ransac dfn configuration file path; \n \
(ii) the essential matrix decomposition dfn configuration file path; \n \
(iii) the reconstruction dfn configuration file path; \n \
(iv) the input correspondences file path, in opencv xml file format; \n \
(v) the horizontal field of view of the camera expressed in meters. \n \n \
(vi) the vertical field of view of the camera expressed in meters. \n \n \
Example Usage: ./ransac_essential_triangulation_test ../tests/ConfigurationFiles/DFNs/CameraTransformEstimation/EssentialMatrixDecomposition_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/PointCloudReconstruction2Dto3D/Triangulation_DevonIsland.yaml ../tests/Data/Images/DevonIslandKeypointsMatches.yaml ../tests/Data/Images/DevonIslandFundamentalMatrix.yaml ../tests/Data/Images/DevonIslandCameraMatrix.xml \n \n";

float ExtractFieldOfViewX(char* argument)
	{
	const std::string errorMessage = "The 5th parameter fieldOfViewX has to be a positive floating point number";
	float fieldOfViewX;

	try 
		{
		fieldOfViewX = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(fieldOfViewX >0, errorMessage);
	
	return fieldOfViewX;
	}

float ExtractFieldOfViewY(char* argument)
	{
	const std::string errorMessage = "The 6th parameter fieldOfViewY has to be a positive floating point number";
	float fieldOfViewY;

	try 
		{
		fieldOfViewY = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(fieldOfViewY >0, errorMessage);
	
	return fieldOfViewY;
	}

int main(int argc, char** argv)
	{
	std::string fundamentalMatrixEstimatorFilePath;
	std::string poseEstimatorConfigurationFilePath;
	std::string reconstructorConfigurationFilePath;
	std::string inputCorrespodencesFilePath;
	float fieldOfViewX, fieldOfViewY;

	ASSERT(argc == 7, USAGE);
	fundamentalMatrixEstimatorFilePath = argv[1];
	poseEstimatorConfigurationFilePath = argv[2];
	reconstructorConfigurationFilePath = argv[3];
	inputCorrespodencesFilePath = argv[4];
	fieldOfViewX = ExtractFieldOfViewX(argv[5]);
	fieldOfViewY = ExtractFieldOfViewY(argv[6]);

	ReconstructionTester tester;
	tester.SetConfigurationFilePaths(fundamentalMatrixEstimatorFilePath, poseEstimatorConfigurationFilePath, reconstructorConfigurationFilePath);
	tester.SetInputFilePath(inputCorrespodencesFilePath);

	FundamentalMatrixRansac* ransac = new FundamentalMatrixRansac();
	EssentialMatrixDecomposition* decomposition = new EssentialMatrixDecomposition();
	Triangulation* triangulation = new Triangulation();
	tester.SetDfns(ransac, decomposition, triangulation);
	
	tester.ExecuteDfns();
	bool success = tester.AreTriangulatedPointsValid(fieldOfViewX, fieldOfViewY);

	VERIFY_REQUIREMENT(success, "Triangulation point validity requirement 4.1.1.5 failed on the input correspondences");
	return 0;
	}


/** @} */
