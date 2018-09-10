/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_POINTCLOUDASSEMBLYINTERFACE_HPP
#define POINTCLOUDASSEMBLY_POINTCLOUDASSEMBLYINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Pointcloud.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that combines two point clouds together
     */
    class PointCloudAssemblyInterface : public DFNCommonInterface
    {
        public:

            PointCloudAssemblyInterface();
            virtual ~PointCloudAssemblyInterface();

            /**
             * Send value to input port "firstPointCloud"
             * @param firstPointCloud: first input point cloud
             */
            virtual void firstPointCloudInput(const asn1SccPointcloud& data);

            /**
             * Send value to input port "secondPointCloud"
             * @param secondPointCloud: second input point cloud
             */
            virtual void secondPointCloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "assembledCloud"
             * @return assembledCloud: point cloud obtained by the combination of the two inputs.
             */
            virtual const asn1SccPointcloud& assembledCloudOutput() const;

        protected:

            asn1SccPointcloud inFirstPointCloud;
            asn1SccPointcloud inSecondPointCloud;
            asn1SccPointcloud outAssembledPointCloud;
    };
}
}

#endif // POINTCLOUDASSEMBLY_POINTCLOUDASSEMBLYINTERFACE_HPP

/** @} */
