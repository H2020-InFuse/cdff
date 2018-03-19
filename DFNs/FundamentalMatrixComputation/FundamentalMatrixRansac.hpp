/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixRansac.hpp
 * @date 01/26/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN applies the RANSAC estimation provided by OpenCV.
 * 
 * This DFN implementation requires the following parameters:
 * @param outlierThreshold, the pixel distance that determines whether an input point (represented by a match between two 2d points) is an outlier in the RANSAC model; Recommended values are 1, 2, and 3. 
 * @param confidence, the minimum acceptable probability that a RANSAC model represents the set of points defined by the correspondence set.
 *
 * @{
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Fundamental Matrix Computation with the Ransac (Randome Sample Consensus) Method.
 *  
 *
 * @{
 */

#ifndef FUNDAMENTAL_MATRIX_RANSAC_HPP
#define FUNDAMENTAL_MATRIX_RANSAC_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Matrix.hpp>
#include <BaseTypes.hpp>
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
    class FundamentalMatrixRansac : public FundamentalMatrixComputationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FundamentalMatrixRansac();
            ~FundamentalMatrixRansac();
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

		struct FundamentalMatrixRansacOptionsSet
			{
			double outlierThreshold; // in pixels
			double confidence; //probability between 0 and 1
			};

		Helpers::ParametersListHelper parametersHelper;
		FundamentalMatrixRansacOptionsSet parameters;
		static const FundamentalMatrixRansacOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeFundamentalMatrix(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);
		BaseTypesWrapper::Point2DConstPtr ComputeSecondEpipole
			(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector, cv::Mat fundamentalMatrix);
		void Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector);
		MatrixWrapper::Matrix3dConstPtr Convert(cv::Mat matrix);

		void ValidateParameters();
		void ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);
    };
}
#endif
/* FundamentalMatrixRansac.hpp */
/** @} */
