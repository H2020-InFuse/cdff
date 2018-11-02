/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DEPTHFILTERING_EXECUTOR_HPP
#define DEPTHFILTERING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <DepthFiltering/DepthFilteringInterface.hpp>
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>

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

void Execute(DepthFilteringInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr& outputFrame);
void Execute(DepthFilteringInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr outputFrame);
void Execute(DepthFilteringInterface* dfn, const FrameWrapper::Frame& inputFrame, FrameWrapper::FrameConstPtr& outputFrame);
void Execute(DepthFilteringInterface* dfn, const FrameWrapper::Frame& inputFrame, FrameWrapper::Frame& outputFrame);

}
}
}

#endif // DEPTHFILTERING_EXECUTOR_HPP

/** @} */
