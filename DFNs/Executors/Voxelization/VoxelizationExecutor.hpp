/**
 * @addtogroup DFNs
 * @{
 */

#ifndef VOXELIZATION_EXECUTOR_HPP
#define VOXELIZATION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <Voxelization/VoxelizationInterface.hpp>
#include <Frame.h>
#include <Frame.hpp>

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
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(VoxelizationInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, asn1SccOctree & outputOctree);
void Execute(VoxelizationInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, const asn1SccOctree * outputOctree);
void Execute(VoxelizationInterface* dfn, const FrameWrapper::Frame& inputFrame, const asn1SccOctree * outputOctree);
void Execute(VoxelizationInterface* dfn, const FrameWrapper::Frame& inputFrame, asn1SccOctree& outputOctree);

}
}
}

#endif // VOXELIZATION_EXECUTOR_HPP

/** @} */
