/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_EXECUTOR_HPP
#define FORCEMESHGENERATOR_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <ForceMeshGenerator/ForceMeshGeneratorInterface.hpp>
#include <PointCloud.hpp>
#include <Sequences.h>

namespace CDFF
{
namespace DFN
{
namespace Executors
{
/**
* All the methods in this file execute the DFN for the filtering of depth images. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFrame: input image;
* @param outputVector: output filtered image.
*
* The main difference between the methods are input and output types
*/
            void Execute(
                    ForceMeshGeneratorInterface* dfn,
                    const asn1SccPose &armBasePose,
                    const asn1SccPose &armEndEffectorPose,
                    const asn1SccWrench &armEndEffectorWrench,
                    PointCloudWrapper::PointCloudPtr outputPointCloud
                    );

            void Execute(
                    ForceMeshGeneratorInterface* dfn,
                    const asn1SccPose &armBasePose,
                    const asn1SccPose &armEndEffectorPose,
                    const asn1SccWrench &armEndEffectorWrench,
                    PointCloudWrapper::PointCloud & outputPointCloud);

}
}
}

#endif // FORCEMESHGENERATOR_EXECUTOR_HPP

/** @} */
