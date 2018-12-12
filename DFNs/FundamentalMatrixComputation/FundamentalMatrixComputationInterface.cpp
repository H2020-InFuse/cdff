/**
 * @addtogroup DFNs
 * @{
 */

#include "FundamentalMatrixComputationInterface.hpp"

namespace CDFF
{
namespace DFN
{

FundamentalMatrixComputationInterface::FundamentalMatrixComputationInterface() :
outFundamentalMatrix(),
outSuccess(),
outInlierMatches()
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

const asn1SccCorrespondenceMap2D& FundamentalMatrixComputationInterface::inlierMatchesOutput() const
{
    return outInlierMatches;
}

}
}

/** @} */
