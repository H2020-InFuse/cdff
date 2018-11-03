/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP
#define PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>
#include <Types/C/Sequences.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that finds primitives in a 2D image
     */
    class PrimitiveMatchingInterface : public DFNCommonInterface
    {
        public:

            PrimitiveMatchingInterface();
            virtual ~PrimitiveMatchingInterface();

            /**
             * Send value to input port "image"
             * @param image
             *     2D image captured by a camera
             */
            virtual void imageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "primitives"
             * @param primitives
             *     string array with the names of the primitives to be searched for in the input image
             */
            virtual void primitivesInput(const asn1SccStringSequence& data);

            /**
             * Query value from output port "image"
             * @return image
             *     same image, with the contour that was matched to a primitive
             */
            virtual const asn1SccFrame& imageOutput() const;
            /**
             * Query value from output port "primitives"
             * @return primitives
             *     string array with the names of the primitives ordered by matching probability
             */
            virtual const asn1SccStringSequence& primitivesOutput() const;

        protected:

            asn1SccFrame inImage;
            asn1SccStringSequence inPrimitives;
            asn1SccFrame outImage;
            asn1SccStringSequence outPrimitives;
    };
}
}

#endif // PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

/** @} */
