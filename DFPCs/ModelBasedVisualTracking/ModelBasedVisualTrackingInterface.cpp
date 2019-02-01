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
ModelBasedVisualTrackingInterface::ModelBasedVisualTrackingInterface()
	{
	asn1SccFrame_Initialize(&inImageLeft);
	asn1SccFrame_Initialize(&inImageRight);
	asn1SccRigidBodyState_Initialize(&inInit);
	asn1SccTime_Initialize(&inImageTime);
	asn1SccTime_Initialize(&inInitTime);
	inDoInit = false;
	asn1SccRigidBodyState_Initialize(&inEgoMotion);
	asn1SccRigidBodyState_Initialize(&outState);
	outSuccess =false;
	
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

const asn1SccRigidBodyState ModelBasedVisualTrackingInterface::stateOutput() const 
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
