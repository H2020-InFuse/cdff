/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITY_HPP
#define DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITY_HPP

#include "DisparityToPointCloudWithIntensityInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloudWithIntensity
{
    /**
     * @brief A class to generate a colored pointcloud from disparity and intensity images
     */
    class DisparityToPointCloudWithIntensity : public DisparityToPointCloudWithIntensityInterface
    {
        public:

            DisparityToPointCloudWithIntensity();
            virtual ~DisparityToPointCloudWithIntensity();

            virtual void configure();
            virtual void process();

        private:
        
            template <typename T>
            bool disp2ptcloudwithintensity(asn1SccFrame &disp, asn1SccFrame &img, asn1SccPointcloud &ptCloud);
    };
}
}
}

#endif // DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITY_HPP

/** @} */
