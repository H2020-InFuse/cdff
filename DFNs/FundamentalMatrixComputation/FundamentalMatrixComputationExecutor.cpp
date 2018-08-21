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
FundamentalMatrixComputationExecutor::FundamentalMatrixComputationExecutor(FundamentalMatrixComputationInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "FundamentalMatrixComputationExecutor: null dfn in input");
	}

FundamentalMatrixComputationExecutor::~FundamentalMatrixComputationExecutor()
	{

	}

void FundamentalMatrixComputationExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success)
	{
	Execute(*inputMatches, outputMatrix, success);
	}

void FundamentalMatrixComputationExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success)
	{
	ASSERT(outputMatrix != NULL, "FundamentalMatrixComputationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *outputMatrix, success);
	}

void FundamentalMatrixComputationExecutor::Execute(const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success)
	{
	ASSERT( outputMatrix == NULL, "FundamentalMatrixComputationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputMatrix = & ( dfn->fundamentalMatrixOutput() );
	success = dfn->successOutput();
	}

void FundamentalMatrixComputationExecutor::Execute(const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success)
	{
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->fundamentalMatrixOutput(), outputMatrix);
	success = dfn->successOutput();
	}

void FundamentalMatrixComputationExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success, CorrespondenceMap2DConstPtr& outputInlierMatches)
	{
	Execute(*inputMatches, outputMatrix, success, outputInlierMatches);
	}

void FundamentalMatrixComputationExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success, CorrespondenceMap2DPtr outputInlierMatches)
	{
	ASSERT(outputMatrix != NULL && outputInlierMatches != NULL, "FundamentalMatrixComputationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *outputMatrix, success, *outputInlierMatches);
	}

void FundamentalMatrixComputationExecutor::Execute(const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success, CorrespondenceMap2DConstPtr& outputInlierMatches)
	{
	ASSERT( outputMatrix == NULL, "FundamentalMatrixComputationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputMatrix = & ( dfn->fundamentalMatrixOutput() );
	success = dfn->successOutput();
	outputInlierMatches = & ( dfn->inlierMatchesOutput() );
	}

void FundamentalMatrixComputationExecutor::Execute(const CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success, CorrespondenceMap2D& outputInlierMatches)
	{
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->fundamentalMatrixOutput(), outputMatrix);
	success = dfn->successOutput();
	Copy( dfn->inlierMatchesOutput(), outputInlierMatches);
	}

}
}

/** @} */
