/**
 * @addtogroup DFNs
 * @{
 */

#ifndef LINEMODTRAIN_LINEMODTRAININTERFACE_HPP
#define LINEMODTRAIN_LINEMODTRAININTERFACE_HPP

#include "DFNCommonInterface.hpp"

namespace CDFF
{
namespace DFN
{

/**
 * DFN that performs Linemod training for object pose detection
 */
class LinemodTrainInterface : public DFNCommonInterface
{
    public:
        LinemodTrainInterface();
        virtual ~LinemodTrainInterface();
};

}
}

#endif // LINEMODTRAIN_LINEMODTRAININTERFACE_HPP

/** @} */
