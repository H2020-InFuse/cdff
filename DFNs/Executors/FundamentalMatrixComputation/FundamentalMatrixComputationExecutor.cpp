/**
 * @addtogroup DFNs
 * @{
 */

#include "FundamentalMatrixComputationExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FundamentalMatrixComputationInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success)
	{
	Execute(dfn, *inputMatches, outputMatrix, success);
	}

void Execute(FundamentalMatrixComputationInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success)
	{
	ASSERT(outputMatrix != NULL, "FundamentalMatrixComputationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *outputMatrix, success);
	}

void Execute(FundamentalMatrixComputationInterface* dfn, const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success)
	{
	ASSERT( dfn!= NULL, "FundamentalMatrixComputationExecutor, input dfn is null");
	ASSERT( outputMatrix == NULL, "FundamentalMatrixComputationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputMatrix = & ( dfn->fundamentalMatrixOutput() );
	success = dfn->successOutput();
	}

void Execute(FundamentalMatrixComputationInterface* dfn, const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success)
	{
	ASSERT( dfn!= NULL, "FundamentalMatrixComputationExecutor, input dfn is null");
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->fundamentalMatrixOutput(), outputMatrix);
	success = dfn->successOutput();
	}

void Execute(FundamentalMatrixComputationInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success, 
	CorrespondenceMap2DConstPtr& outputInlierMatches)
	{
	Execute(dfn, *inputMatches, outputMatrix, success, outputInlierMatches);
	}

void Execute(FundamentalMatrixComputationInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success, 
	CorrespondenceMap2DPtr outputInlierMatches)
	{
	ASSERT(outputMatrix != NULL && outputInlierMatches != NULL, "FundamentalMatrixComputationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *outputMatrix, success, *outputInlierMatches);
	}

void Execute(FundamentalMatrixComputationInterface* dfn, const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success, 
	CorrespondenceMap2DConstPtr& outputInlierMatches)
	{
	ASSERT( dfn!= NULL, "FundamentalMatrixComputationExecutor, input dfn is null");
	ASSERT( outputMatrix == NULL && outputInlierMatches == NULL, "FundamentalMatrixComputationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputMatrix = & ( dfn->fundamentalMatrixOutput() );
	success = dfn->successOutput();
	outputInlierMatches = & ( dfn->inlierMatchesOutput() );
	}

void Execute(FundamentalMatrixComputationInterface* dfn, const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success, 
	CorrespondenceMap2D& outputInlierMatches)
	{
	ASSERT( dfn!= NULL, "FundamentalMatrixComputationExecutor, input dfn is null");
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->fundamentalMatrixOutput(), outputMatrix);
	success = dfn->successOutput();
	Copy( dfn->inlierMatchesOutput(), outputInlierMatches);
	}

}
}
}

/** @} */
