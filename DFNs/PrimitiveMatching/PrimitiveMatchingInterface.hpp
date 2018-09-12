/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP
#define PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <BaseTypes.hpp>


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
            * Send value to input port "primitives"
            * @return string array: array of strings with the primitives to be matched
            */
            virtual void primitiveArrayInput(const BaseTypesWrapper::asn1SccT_StringArray& data);

             /**
             * Query value from output port "primitives"
             * @return string array: array of strings with the input primitives ordered by matching probabilities
             */
            virtual BaseTypesWrapper::asn1SccT_StringArray primitivesMatchedOutput() const;

            /**
            * Query value from output port "image"
            * @return image: input image with the matched contour
            */
            virtual const asn1SccFrame& imageWithMatchedContourOutput() const;

    protected:

            asn1SccFrame inImage;
            BaseTypesWrapper::asn1SccT_StringArray inPrimitiveArray;
            asn1SccFrame outImageWithMatchedContour;
            BaseTypesWrapper::asn1SccT_StringArray outPrimitiveArrayMatched;

    };
}
}

#endif // PRIMITIVEMATCHING_PRIMITIVEMATCHINGINTERFACE_HPP

/** @} */
