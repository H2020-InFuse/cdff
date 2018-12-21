/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP
#define STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP

#include "StereoMotionEstimationInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoMotionEstimation
{
    /**
     * TODO Class documentation
     */
    class StereoMotionEstimationEdres : public StereoMotionEstimationInterface
    {
        public:

            StereoMotionEstimationEdres();
            virtual ~StereoMotionEstimationEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP

/** @} */
