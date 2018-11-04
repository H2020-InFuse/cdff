/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDFILTERING_POINTCLOUDTRANSFORMINTERFACE_HPP
#define POINTCLOUDFILTERING_POINTCLOUDTRANSFORMINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that applies a filter on a point cloud
     */
    class PointCloudFilteringInterface : public DFNCommonInterface
    {
        public:

            PointCloudFilteringInterface();
            virtual ~PointCloudFilteringInterface();

            /**
             * Send value to input port "pointCloud"
             * @param pointCloud: input point cloud
             */
            virtual void pointCloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "filteredPointCloud"
             * @return filteredPointCloud: point cloud after filter is applied
             */
            virtual const asn1SccPointcloud& filteredPointCloudOutput() const;

        protected:

            asn1SccPointcloud inPointCloud;
            asn1SccPointcloud outFilteredPointCloud;
    };
}
}

#endif // POINTCLOUDFILTERING_POINTCLOUDTRANSFORMINTERFACE_HPP

/** @} */
