/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DEPTHFILTERING_DEPTHFILTERINGINTERFACE_HPP
#define DEPTHFILTERING_DEPTHFILTERINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>


namespace CDFF
{
namespace DFN
{
    /**
     * DFN that filters outliers in a 2D depth map
     */
    class DepthFilteringInterface : public DFNCommonInterface
    {
        public:

            DepthFilteringInterface();

            /**
             * Send value to input port
             * @param frame: 2D depth image
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
             * Query value from output port
             * @return filtered 2D depth image
             */
            virtual const asn1SccFrame& frameOutput() const;

        protected:

            asn1SccFrame inFrame;
            asn1SccFrame outFrame;
    };
}
}

#endif // DEPTHFILTERING_DEPTHFILTERINGINTERFACE_HPP

/** @} */
