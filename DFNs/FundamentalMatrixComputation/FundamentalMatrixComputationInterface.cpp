/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixComputationInterface.cpp
 * @date 26/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FundamentalMatrixComputationInterface class
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
#include "FundamentalMatrixComputationInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FundamentalMatrixComputationInterface::FundamentalMatrixComputationInterface()
	{
	
	}

FundamentalMatrixComputationInterface::~FundamentalMatrixComputationInterface()
	{

	}

void FundamentalMatrixComputationInterface::correspondenceMapInput(CorrespondenceMap2DConstPtr data) 
	{
    	inCorrespondenceMap = data;
	}

Matrix3dConstPtr FundamentalMatrixComputationInterface::fundamentalMatrixOutput() 
	{
    	return outFundamentalMatrix;
	}

Point2DConstPtr FundamentalMatrixComputationInterface::secondEpipoleOutput()
	{
	return outSecondEpipole;
	}

bool FundamentalMatrixComputationInterface::successOutput()
	{
	return outSuccess;
	}




}

/** @} */

