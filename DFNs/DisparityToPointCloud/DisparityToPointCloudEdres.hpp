/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDEDRES_HPP
#define DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDEDRES_HPP

#include "DisparityToPointCloudInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloud
{
    /**
     * @brief Implementation of the disparity to pointcloud algorithms provided by EDRES library
     */
    class DisparityToPointCloudEdres : public DisparityToPointCloudInterface
    {
        public:

            DisparityToPointCloudEdres();
            virtual ~DisparityToPointCloudEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUDEDRES_HPP

/** @} */
