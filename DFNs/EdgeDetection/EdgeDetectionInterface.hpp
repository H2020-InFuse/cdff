/**
 * @addtogroup DFNs
 * @{
 */

#ifndef EDGEDETECTION_INTERFACE_HPP
#define EDGEDETECTION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>

namespace dfn_ci
{
    /**
     * DFN that applies an edge detection filter to a 2D image
     */
    class EdgeDetectionInterface : public DFNCommonInterface
    {
        public:

            EdgeDetectionInterface();
            virtual ~EdgeDetectionInterface();

            /**
             * Send value to input port "image"
             * @param image: 2D image
             */
            virtual void imageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "edgeMap"
             * @return edgeMap: edge map of the input image
             */
            virtual const asn1SccFrame& edgeMapOutput() const;
            /**
             * Query value from output port "sobelGradientX"
             * @return sobelGradientX: image gradient in the x direction
             */
            virtual const asn1SccFrame& sobelGradientXOutput() const;
            /**
             * Query value from output port "sobelGradientY"
             * @return sobelGradientY: image gradient in the y direction
             */
            virtual const asn1SccFrame& sobelGradientYOutput() const;

        protected:

            asn1SccFrame inImage;
            asn1SccFrame outEdgeMap;
            asn1SccFrame outSobelGradientX;
            asn1SccFrame outSobelGradientY;
    };
}

#endif // EDGEDETECTION_INTERFACE_HPP

/** @} */
