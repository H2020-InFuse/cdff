/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYEDRES_HPP
#define DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYEDRES_HPP

#include "DisparityToPointCloudWithIntensityInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloudWithIntensity
{

    /**
     * @brief Implementation of the disparity to pointcloud with intensity algorithms provided by EDRES library
     * 
     */
    class DisparityToPointCloudWithIntensityEdres : public DisparityToPointCloudWithIntensityInterface
    {
        public:

            DisparityToPointCloudWithIntensityEdres();
            virtual ~DisparityToPointCloudWithIntensityEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // DISPARITYTOPOINTCLOUDWITHINTENSITY_DISPARITYTOPOINTCLOUDWITHINTENSITYEDRES_HPP

/** @} */
