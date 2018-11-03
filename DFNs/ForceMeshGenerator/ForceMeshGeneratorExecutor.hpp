/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_EXECUTOR_HPP
#define FORCEMESHGENERATOR_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "ForceMeshGeneratorInterface.hpp"
#include <Types/CPP/PointCloud.hpp>
#include <Sequences.h>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the filtering of depth images. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFrame: input image;
* @param outputVector: output filtered image.
*
* The main difference between the methods are input and output types
*/
    class ForceMeshGeneratorExecutor
    {
        public:

            explicit ForceMeshGeneratorExecutor(ForceMeshGeneratorInterface* dfn);

            void Execute(
                    const asn1SccPose &armBasePose,
                    const asn1SccPose &armEndEffectorPose,
                    const asn1SccWrench &armEndEffectorWrench,
                    PointCloudWrapper::PointCloudPtr outputPointCloud
                    );

            void Execute(
                    const asn1SccPose &armBasePose,
                    const asn1SccPose &armEndEffectorPose,
                    const asn1SccWrench &armEndEffectorWrench,
                    PointCloudWrapper::PointCloud & outputPointCloud);

        private:

            ForceMeshGeneratorInterface* dfn;
    };
}
}

#endif // FORCEMESHGENERATOR_EXECUTOR_HPP

/** @} */
