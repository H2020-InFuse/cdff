/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionTester.hpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.5 of deliverable 5.5.
 * "Triangulated points are guaranteed by the algorithm to be placed within the field of view of the camera."
 *
 * @{
 */

#ifndef RECONSTRUCTION_TESTER_HPP
#define RECONSTRUCTION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/Pose.hpp>
#include <Types/CPP/Matrix.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/PointCloud.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <boost/make_shared.hpp>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class ReconstructionTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ReconstructionTester();
		~ReconstructionTester();

		void SetDfns(CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixEstimator, 
				CDFF::DFN::CamerasTransformEstimationInterface* poseEstimator, CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor);
		void SetConfigurationFilePaths(const std::string& fundamentalMatrixEstimatorFilePath, const std::string& poseEstimatorConfigurationFilePath, 
			const std::string& reconstructorConfigurationFilePath);
		void SetInputFilePath(const std::string& inputCorrespodencesFilePath);
		void ExecuteDfns();
		bool AreTriangulatedPointsValid(float fieldOfViewX, float fieldOfViewY);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		std::string poseEstimatorConfigurationFilePath;
		std::string reconstructorConfigurationFilePath;
		std::string fundamentalMatrixEstimatorFilePath;
		std::string inputCorrespodencesFilePath;

		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap;
		MatrixWrapper::Matrix3dConstPtr fundamentalMatrix;
		bool fundamentalMatrixSuccess;
		PoseWrapper::Transform3DConstPtr cameraTransform;
		bool poseEstimatorWasSuccessful;
		PointCloudWrapper::PointCloudConstPtr outputPointCloud;

		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixEstimator;
		CDFF::DFN::CamerasTransformEstimationInterface* poseEstimator;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor;

		bool dfnsWereLoaded;
		bool inputCorrespondencesWereLoaded;
		bool dfnsWereExecuted;

		void LoadInputCorrespondences();
		void ConfigureDfns();
		bool AllPointsAreInTheFieldOfView(float fieldOfViewX, float fieldOfViewY);
	};

#endif

/* ReconstructionTester.hpp */
/** @} */
