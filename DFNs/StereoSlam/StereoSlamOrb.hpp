/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOSLAM_STEREOSLAMORB_HPP
#define STEREOSLAM_STEREOSLAMORB_HPP

#include "StereoSlamInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoSlam
{
    /**
     * TODO Class documentation
     */
    class StereoSlamOrb : public StereoSlamInterface
    {
        public:

            StereoSlamOrb();
            virtual ~StereoSlamOrb();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // STEREOSLAM_STEREOSLAMORB_HPP

/** @} */
