/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP
#define PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <Sequences.h>


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
             * Send value to input port "inputImage"
             * @param image: 2D image captured by a camera with the ROI already segmented
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
            * Send value to input port "inputPrimitives"
            * @return string array: array of strings with the primitives to be matched
            */
            virtual void primitiveSequenceInput(const asn1SccStringSequence & data);

             /**
             * Query value from output port "outputPrimitives"
             * @return string array: array of strings with the input primitives ordered by matching probabilities
             */
            virtual const asn1SccStringSequence & primitivesMatchedOutput() const;

            /**
            * Query value from output port "outputImage"
            * @return image: input image with the matched contour
            */
            virtual const asn1SccFrame& imageWithMatchedContourOutput() const;

    protected:

            asn1SccFrame inImage;
            asn1SccStringSequence inPrimitiveSequence;
            asn1SccFrame outImageWithMatchedContour;
            asn1SccStringSequence outPrimitiveSequenceMatched;

    };
}
}

#endif // PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

/** @} */
