/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYFILTERING_DISPARITYFILTERINGINTERFACE_HPP
#define DISPARITYFILTERING_DISPARITYFILTERINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that filters a disparity image
     */
    class DisparityFilteringInterface : public DFNCommonInterface
    {
        public:

            DisparityFilteringInterface();
            virtual ~DisparityFilteringInterface();

            /**
             * Send value to input port "rawDisparity"
             * @param rawDisparity
             *     Raw disparity image
             */
            virtual void rawDisparityInput(const asn1SccFrame& data);

            /**
             * Query value from output port "filteredDisparity"
             * @return filteredDisparity
             *     Filtered disparity image
             */
            virtual const asn1SccFrame& filteredDisparityOutput() const;

        protected:

            asn1SccFrame inRawDisparity;
            asn1SccFrame outFilteredDisparity;
    };
}
}

#endif // DISPARITYFILTERING_DISPARITYFILTERINGINTERFACE_HPP

/** @} */
