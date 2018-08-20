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
/**
* All the methods in this class execute the DFN for the computation of the fundamental matrix. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputMatches: input matches between 2d features;
* @param outputMatrix: the output fundamental matrix;
* @param success: output boolean telling whether the computation was successfull.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
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
