/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FUNDAMENTALMATRIXCOMPUTATION_EXECUTOR_HPP
#define FUNDAMENTALMATRIXCOMPUTATION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FundamentalMatrixComputationInterface.hpp"
#include <Matrix.hpp>
#include <CorrespondenceMap2D.hpp>

namespace CDFF
{
namespace DFN
{
    class FundamentalMatrixComputationExecutor
    {
        public:

            FundamentalMatrixComputationExecutor(FundamentalMatrixComputationInterface* dfn);
            ~FundamentalMatrixComputationExecutor();

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success);

        private:

            FundamentalMatrixComputationInterface* dfn;
    };
}
}

#endif // FUNDAMENTALMATRIXCOMPUTATION_EXECUTOR_HPP

/** @} */
