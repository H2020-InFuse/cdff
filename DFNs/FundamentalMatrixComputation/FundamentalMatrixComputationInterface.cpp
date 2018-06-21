/**
 * @addtogroup DFNs
 * @{
 */

#include "FundamentalMatrixComputationInterface.hpp"

namespace dfn_ci
{

FundamentalMatrixComputationInterface::FundamentalMatrixComputationInterface()
{
}

FundamentalMatrixComputationInterface::~FundamentalMatrixComputationInterface()
{
}

void FundamentalMatrixComputationInterface::matchesInput(const asn1SccCorrespondenceMap2D& data)
{
    inMatches = data;
}

const asn1SccMatrix3d& FundamentalMatrixComputationInterface::fundamentalMatrixOutput() const
{
    return outFundamentalMatrix;
}

bool FundamentalMatrixComputationInterface::successOutput() const
{
    return outSuccess;
}

}

/** @} */
