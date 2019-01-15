/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDINTERFACE_HPP
#define DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>
#include <Types/C/Pointcloud.h>

namespace CDFF
{
namespace DFN
{
    class DisparityToPointCloudInterface : public DFNCommonInterface
    {
        public:

            DisparityToPointCloudInterface();
            virtual ~DisparityToPointCloudInterface();

            /**
             * Send value to input port "dispImage"
             * @param dispImage
             *     A disparity image
             */
            virtual void dispImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "pointCloud"
             * @return pointCloud
             *     The corresponding pointcloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

        protected:

            asn1SccFrame inDispImage;
            asn1SccPointcloud outPointCloud;
    };
}
}

#endif // DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDINTERFACE_HPP

/** @} */
