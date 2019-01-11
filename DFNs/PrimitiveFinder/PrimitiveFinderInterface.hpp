/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEFINDER_PRIMITIVEFINDERINTERFACE_HPP
#define PRIMITIVEFINDER_PRIMITIVEFINDERINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Sequences.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that finds primitives in a 2D image
     */
    class PrimitiveFinderInterface : public DFNCommonInterface
    {
        public:

            PrimitiveFinderInterface();
            virtual ~PrimitiveFinderInterface();

            /**
             * Send value to input port "image"
             * @param image
             *     2D image captured by a camera
             */
            virtual void imageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "primitive"
             * @param primitive
             *     primitive to be found in the input image
             */
            virtual void primitiveInput(const asn1SccT_String& data);

            /**
             * Query value from output port "primitives"
             * @return primitives
             *     location and necessary information of the found primitives
             */
            virtual const asn1SccVectorXdSequence& primitivesOutput() const;

        protected:

            asn1SccFrame inImage;
            asn1SccT_String inPrimitive;
            asn1SccVectorXdSequence outPrimitives;
    };
}
}

#endif // PRIMITIVEFINDER_PRIMITIVEFINDERINTERFACE_HPP

/** @} */
