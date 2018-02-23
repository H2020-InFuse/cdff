/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixComputationInterface.hpp
 * @date 26/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that compute a fundamental matrix from a set of 2D correspondences.    
 *
 * @{
 */
#ifndef FUNDAMENTAL_MATRIX_COMPUTATION_INTERFACE_HPP
#define FUNDAMENTAL_MATRIX_COMPUTATION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Matrix.hpp>
#include <BaseTypes.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FundamentalMatrixComputationInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FundamentalMatrixComputationInterface();
            virtual ~FundamentalMatrixComputationInterface();
            /**
            * Send value to input port image
            * @param correspondenceSet, this is a list coordinates of physical points in two distinct images taken from a camera. The set contains pairs of 2d coordinates ( (x1, y1), (x2, y2)), 
            * each pair represents the same physical point. (x1, y1) is its position in the first camera image and (x2, y2) is its position in the second camera image.
            */
            virtual void correspondenceMapInput(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr data);

            /**
            * Receive value from output port featuresSet
            * @param fundamentalMatrix, This is the fundamental matrix of the camera that took the two images.
            */
            virtual MatrixWrapper::Matrix3dConstPtr fundamentalMatrixOutput();

            /**
            * Receive value from output port featuresSet
            * @param success, This outputs tells whether the matrix computation was succesfull. The computation may fail if the inputs are not good. 
	    * If the computation fails the first output is meaningless.
            */
            virtual bool successOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inCorrespondenceMap;
            MatrixWrapper::Matrix3dConstPtr outFundamentalMatrix;
	    bool outSuccess;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FundamentalMatrixComputationInterface.hpp */
/** @} */
