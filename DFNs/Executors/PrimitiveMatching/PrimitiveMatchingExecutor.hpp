/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_EXECUTOR_HPP
#define PRIMITIVEMATCHING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <PrimitiveMatching/PrimitiveMatchingInterface.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/C/Sequences.h>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

/**
* All the methods in this file execute the DFN for primitive matching. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFrame: input image;
* @param outputPrimitive: output string with the name of the matched primitive.
*
*/

void Execute(PrimitiveMatchingInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, const asn1SccStringSequence& inputPrimitiveArray, asn1SccStringSequence& outputPrimitiveArray);
void Execute(PrimitiveMatchingInterface* dfn, const FrameWrapper::Frame& inputFrame, const asn1SccStringSequence& inputPrimitiveArray, asn1SccStringSequence& outputPrimitiveArray);

}
}
}

#endif // PRIMITIVEMATCHING_EXECUTOR_HPP

/** @} */
