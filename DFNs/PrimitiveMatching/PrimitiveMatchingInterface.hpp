/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP
#define PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>

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
             * @param image: 2D image captured by a camera with the ROI already segmented
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
             * Query value from output port "primitives"
             * @return string: string with the name of the matched primitive
             */
            virtual asn1SccT_String primitiveMatchedOutput() const;

            /**
            * Query value from output port "image"
            * @return image: input image with the matched contour
            */
            virtual const asn1SccFrame& imageWithMatchedContourOutput() const;

    protected:

            asn1SccFrame inImage;
            asn1SccFrame outImageWithMatchedContour;
            asn1SccT_String outPrimitiveMatched;

    };
}
}

#endif // PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

/** @} */
