/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYINTERFACE_HPP
#define DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    class DisparityToPointCloudWithIntensityInterface : public DFNCommonInterface
    {
        public:

            DisparityToPointCloudWithIntensityInterface();
            virtual ~DisparityToPointCloudWithIntensityInterface();

            /**
             * Send value to input port "dispImage"
             * @param dispImage
             *     A disparity image
             */
            virtual void dispImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "intensityImage"
             * @param intensityImage
             *     Left image for intensity information
             */
            virtual void intensityImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "pointCloud"
             * @return pointCloud
             *     The corresponding pointcloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

        protected:

            asn1SccFrame inDispImage;
            asn1SccFrame inIntensityImage;
            asn1SccPointcloud outPointCloud;
    };
}
}

#endif // DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYINTERFACE_HPP

/** @} */
