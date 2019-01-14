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

namespace CDFF
{
namespace DFPC
{

//using namespace FrameWrapper;
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ModelBasedVisualTrackingInterface::ModelBasedVisualTrackingInterface():

	 inImageLeft(),
	 inImageRight(),
	 inInit(),
	 inImageTime(),
	 inInitTime(),
	 inDoInit(),
	 inEgoMotion(),
         outState(),
	 outSuccess()
	{
	
	}

ModelBasedVisualTrackingInterface::~ModelBasedVisualTrackingInterface()
	{

	}

void ModelBasedVisualTrackingInterface::imageLeftInput(const asn1SccFrame& data)
	{
    	inImageLeft = data;
	}
void ModelBasedVisualTrackingInterface::imageRightInput(const asn1SccFrame& data)
	{
    	inImageRight = data;
	}
void ModelBasedVisualTrackingInterface::imageTimeInput(const asn1SccTime& data)
	{
	inImageTime = data;
	}
void ModelBasedVisualTrackingInterface::initTimeInput(const asn1SccTime& data)
	{
	 inInitTime = data;
	}

void ModelBasedVisualTrackingInterface::initInput(const asn1SccRigidBodyState& data)
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

asn1SccRigidBodyState ModelBasedVisualTrackingInterface::stateOutput() const 
	{
    	return outState;
	}

bool ModelBasedVisualTrackingInterface::successOutput() const
	{
	return outSuccess;
	}
}
}

/** @} */
