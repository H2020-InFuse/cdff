/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixComputation.hpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Cameras Transform Estimation though computation of the essential matrix from the fundamental matrix.
 *  
 *
 * @{
 */

#ifndef ESSENTIAL_MATRIX_COMPUTATION_HPP
#define ESSENTIAL_MATRIX_COMPUTATION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class EssentialMatrixComputation : public CamerasTransformEstimationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            EssentialMatrixComputation();
            ~EssentialMatrixComputation();
            void process();
            void configure();

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
		Helpers::ParametersListHelper parametersHelper;

		struct CameraMatrix
			{
			double focalLengthX;
			double focalLengthY;
			cv::Point2d principlePoint;
			};

		struct EssentialMatrixComputationOptionsSet
			{
			int numberOfTestPoints;
			CameraMatrix firstCameraMatrix;
			CameraMatrix secondCameraMatrix;
			};

		cv::Mat firstCameraMatrix;
		cv::Mat secondCameraMatrix;

		EssentialMatrixComputationOptionsSet parameters;
		static const EssentialMatrixComputationOptionsSet DEFAULT_PARAMETERS;
		cv::Mat ConvertToMat(CameraMatrix cameraMatrix);
		cv::Mat ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix);
		cv::Mat Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap);

		std::vector<cv::Mat> ComputeTransformMatrix(cv::Mat fundamentalMatrix);
		int FindValidTransform(std::vector<cv::Mat> transformsList, cv::Mat correspondenceMap);

		void ValidateParameters();
		void ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat CorrespondenceMap);
    };
}
#endif
/* EssentialMatrixComputation.hpp */
/** @} */
