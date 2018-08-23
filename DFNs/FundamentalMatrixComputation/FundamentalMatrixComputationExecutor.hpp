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
* All the methods in this class execute the DFN for the computation of the fundamental matrix. A DFN instance has to be passed in the constructor of these class. 
* There are two sets of methods, the first four methods take the following parameters:
* @param inputMatches: input matches between 2d features;
* @param outputMatrix: the output fundamental matrix;
* @param success: output boolean telling whether the computation was successfull.
*
* There the following four methods take the following parameters:
* @param inputMatches: input matches between 2d features;
* @param outputMatrix: the output fundamental matrix;
* @param success: output boolean telling whether the computation was successfull.
* @param outputInlierMatches: the output matches between 2d features whose reprojection error is below a give threshold;
*
* The main difference between each method in a set of four methdos are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
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


	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success,
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputInlierMatches);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, MatrixWrapper::Matrix3dPtr outputMatrix, bool& success,
			CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr outputInlierMatches);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3dConstPtr& outputMatrix, bool& success,
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputInlierMatches);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, MatrixWrapper::Matrix3d& outputMatrix, bool& success,
			CorrespondenceMap2DWrapper::CorrespondenceMap2D& outputInlierMatches);

        private:

            FundamentalMatrixComputationInterface* dfn;
    };
}
}

#endif // FUNDAMENTALMATRIXCOMPUTATION_EXECUTOR_HPP

/** @} */
