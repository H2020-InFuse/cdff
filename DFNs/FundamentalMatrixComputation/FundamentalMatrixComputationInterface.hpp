/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP
#define FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Eigen.h>
#include <CorrespondenceMap2D.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the fundamental matrix of a camera pair based on pairs
     * of 2D matching keypoints found in two images that the cameras captured
     */
    class FundamentalMatrixComputationInterface : public CDFF::DFN::DFNCommonInterface
    {
        public:

            FundamentalMatrixComputationInterface();
            virtual ~FundamentalMatrixComputationInterface();

            /**
             * Send value to input port "matches"
             * @param matches: keypoint matches between two images A and B.
             *        This is a list of pairs of 2D coordinates: ((x^A_1,
             *        y^A_1), (x^B_1, y^B_1)), ..., ((x^A_n, y^A_n), (x^B_n,
             *        y^B_n)). Each pair contains the coordinates of the same
             *        keypoint (hopefully, the same physical point) in each
             *        image.
             */
            virtual void matchesInput(const asn1SccCorrespondenceMap2D& data);

            /**
             * Query value from output port "fundamentalMatrix"
             * @return fundamentalMatrix: fundamental matrix of the camera pair
             *         that captured the two images
             */
            virtual const asn1SccMatrix3d& fundamentalMatrixOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful matrix
             *         computation. Fundamental matrix computation may fail if
             *         the matches are not good enough; in that case, the
             *         returned matrix is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccCorrespondenceMap2D inMatches;
            asn1SccMatrix3d outFundamentalMatrix;
            bool outSuccess;
    };
}
}

#endif // FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP

/** @} */
