/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixRansac.hpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Cameras Transform Estimation though computation of the essential matrix with the Ransac (Random Sample Consensus) Method.
 *  
 *
 * @{
 */

#ifndef ESSENTIAL_MATRIX_RANSAC_HPP
#define ESSENTIAL_MATRIX_RANSAC_HPP

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
    class EssentialMatrixRansac : public CamerasTransformEstimationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            EssentialMatrixRansac();
            ~EssentialMatrixRansac();
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

		struct EssentialMatrixRansacOptionsSet
			{
			double outlierThreshold; // in pixels
			double confidence; //probability between 0 and 1
			CameraMatrix firstCameraMatrix;
			CameraMatrix secondCameraMatrix;
			};

		EssentialMatrixRansacOptionsSet parameters;
		static const EssentialMatrixRansacOptionsSet DEFAULT_PARAMETERS;
		cv::Mat ConvertToMat(CameraMatrix cameraMatrix);

		cv::Mat ComputeTransformMatrix(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);
		cv::Point3d ComputeSecondImageEpipole(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector, cv::Mat fundamentalMatrix);
		cv::Mat ComputeTransformMatrix(cv::Mat fundamentalMatrix, cv::Point3d epipole);
		cv::Mat ComputeEssentialMatrix(cv::Mat fundamentalMatrix);
		void Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector);
		PoseWrapper::Transform3DConstPtr Convert(cv::Mat transformMatrix);

		void ValidateParameters();
		void ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);
    };
}
#endif
/* EssentialMatrixRansac.hpp */
/** @} */
