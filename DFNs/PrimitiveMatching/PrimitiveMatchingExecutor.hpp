/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_EXECUTOR_HPP
#define PRIMITIVEMATCHING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "PrimitiveMatchingInterface.hpp"
#include <Frame.hpp>
#include <Sequences.h>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for primitive matching. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFrame: input image;
* @param outputPrimitive: output string with the name of the matched primitive.
*
*/
    class PrimitiveMatchingExecutor
    {
        public:

            PrimitiveMatchingExecutor(PrimitiveMatchingInterface* dfn);
            ~PrimitiveMatchingExecutor();

            void Execute(FrameWrapper::FrameConstPtr inputFrame, const asn1SccStringSequence& inputPrimitiveArray, asn1SccStringSequence outputPrimitiveArray);
            void Execute(const FrameWrapper::Frame& inputFrame, const asn1SccStringSequence& inputPrimitiveArray, asn1SccStringSequence outputPrimitiveArray);

            private:

                PrimitiveMatchingInterface* dfn;
    };
}
}

#endif // PRIMITIVEMATCHING_EXECUTOR_HPP

/** @} */
