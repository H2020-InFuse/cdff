/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_EXECUTOR_HPP
#define BUNDLEADJUSTMENT_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <BundleAdjustment/BundleAdjustmentInterface.hpp>
#include <Types/CPP/CorrespondenceMaps2DSequence.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Types/CPP/PointCloud.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

/**
* All the methods in this file execute the DFN for the computation of the camera transforms. A DFN instance has to be passed in the constructor of these class. 
* The first four methods take the following parameters:
* @param inputMatches:input 2d matches between 2d features;
* @param outputTransforms: output estimated pose of all the cameras in the reference system of the first;
* @param success: output boolean telling whether the estimation was successfull;
* @param error: output measure of the estimation error.
*
* The second four methods take the following parameters:
* @param inputMatches:input 2d matches between 2d features;
* @param outputTransforms: output estimated pose of all the cameras in the reference system of the first;
* @param poseGuess: initial estimaton of the pose of all the cameras in the reference system of the first;
* @param cloudGuess: initial estimation of the 3d point cloud representing the 2d features in the reference frame of the first camera. 
* @param success: output boolean telling whether the estimation was successfull;
* @param error: output measure of the estimation error.
*
* The main difference within each group of four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN; 
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, 
	bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequencePtr outputTransforms, 
	bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, 
	bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, PoseWrapper::Poses3DSequence& outputTransforms, 
	bool& success, float& error);

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr poseGuess, 
	PointCloudWrapper::PointCloudConstPtr cloudGuess, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr poseGuess, 
	PointCloudWrapper::PointCloudConstPtr cloudGuess, PoseWrapper::Poses3DSequencePtr outputTransforms, bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, const PoseWrapper::Poses3DSequence& poseGuess, const 
	PointCloudWrapper::PointCloud& cloudGuess, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);
void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, const PoseWrapper::Poses3DSequence& poseGuess, const 
	PointCloudWrapper::PointCloud& cloudGuess, PoseWrapper::Poses3DSequence& outputTransforms, bool& success, float& error);

}
}
}

#endif // BUNDLEADJUSTMENT_EXECUTOR_HPP

/** @} */
