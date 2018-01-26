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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>


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

		FundamentalMatrixRansacOptionsSet parameters;
		static const FundamentalMatrixRansacOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeFundamentalMatrix(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);
		void Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector);
		MatrixWrapper::Matrix3dConstPtr Convert(cv::Mat matrix);

		void ValidateParameters();
		void ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* FundamentalMatrixRansac.hpp */
/** @} */
