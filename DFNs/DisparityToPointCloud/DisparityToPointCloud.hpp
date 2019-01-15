/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUD_HPP
#define DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUD_HPP

#include "DisparityToPointCloudInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloud
{
    /**
     * TODO Class documentation
     */
    class DisparityToPointCloud : public DisparityToPointCloudInterface
    {
        public:

            DisparityToPointCloud();
            virtual ~DisparityToPointCloud();

            virtual void configure();
            virtual void process();
        
        private:
            template<typename T>
            bool disp2ptcloud(asn1SccFrame &disp, asn1SccPointcloud &ptCloud);
    };
}
}
}

#endif // DISPARITYTOPOINTCLOUD_DISPARITYTOPOINTCLOUD_HPP

/** @} */
