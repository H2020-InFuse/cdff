/**
 * @addtogroup DFNs
 * @{
 */

#include "FundamentalMatrixComputationInterface.hpp"

namespace CDFF
{
namespace DFN
{

FundamentalMatrixComputationInterface::FundamentalMatrixComputationInterface()
{
    asn1SccCorrespondenceMap2D_Initialize(& inMatches);
    asn1SccMatrix3d_Initialize(& outFundamentalMatrix);
    asn1SccCorrespondenceMap2D_Initialize(& outInlierMatches);
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
