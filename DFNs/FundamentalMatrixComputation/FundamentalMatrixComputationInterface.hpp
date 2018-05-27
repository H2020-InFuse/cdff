/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP
#define FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Eigen.h>
#include <CorrespondenceMap2D.h>

namespace dfn_ci
{
    /**
     * DFN that estimates the fundamental matrix of a camera pair
     * from a set of 2D matches (keypoint pairs)
     */
    class FundamentalMatrixComputationInterface : public DFNCommonInterface
    {
        public:

            FundamentalMatrixComputationInterface();
            virtual ~FundamentalMatrixComputationInterface();

            /**
             * Send value to input port "matches"
             * @param matches: keypoint pairs corresponding to the same point
             *        found in two different images. This is a list of pairs of
             *        2D coordinates: ((x1, y1), (x2, y2)), ... Each pair
             *        references the same physical point: (x1, y1) is its
             *        position in the first image and (x2, y2) is its position
             *        in the second image.
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
             *         the input is not good; in that case, the returned matrix
             *         is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccCorrespondenceMap2D inMatches;
            asn1SccMatrix3d outFundamentalMatrix;
            bool outSuccess;
    };
}

#endif // FUNDAMENTALMATRIXCOMPUTATION_INTERFACE_HPP

/** @} */
