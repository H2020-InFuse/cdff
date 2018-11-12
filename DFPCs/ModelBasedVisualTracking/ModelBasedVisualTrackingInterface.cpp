/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ModelBasedVisualTrackingInterfaceInterface.cpp
 * @date 23/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ModelBasedVisualTrackingInterfaceInterface class
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ModelBasedVisualTrackingInterface.hpp"
#include "Errors/Assert.hpp"

namespace CDFF
{
namespace DFPC
{

using namespace FrameWrapper;
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ModelBasedVisualTrackingInterface::ModelBasedVisualTrackingInterface()
	{
	
	}

ModelBasedVisualTrackingInterface::~ModelBasedVisualTrackingInterface()
	{

	}

void ModelBasedVisualTrackingInterface::imageLeftInput(FrameWrapper::FrameConstPtr data)
	{
    	inImageLeft = data;
	}
void ModelBasedVisualTrackingInterface::imageRightInput(FrameWrapper::FrameConstPtr data)
	{
    	inImageRight = data;
	}
void ModelBasedVisualTrackingInterface::imageTimeInput(asn1SccTime data)
	{
	inImageTime = data;
	}
void ModelBasedVisualTrackingInterface::initTimeInput(const asn1SccTime data)
	{
	 inInitTime = data;
	}

void ModelBasedVisualTrackingInterface::initInput(const asn1SccRigidBodyState &data)
	{
	inInit = data;
	}
void ModelBasedVisualTrackingInterface::doInitInput(bool data)
	{
         inDoInit = data;
	}
void ModelBasedVisualTrackingInterface::egoMotionInput(const asn1SccRigidBodyState& data)
	{
         inEgoMotion = data;
	}

asn1SccRigidBodyState ModelBasedVisualTrackingInterface::stateOutput() 
	{
    	return outState;
	}

bool ModelBasedVisualTrackingInterface::successOutput()
	{
	return outSuccess;
	}
}
}

/** @} */
