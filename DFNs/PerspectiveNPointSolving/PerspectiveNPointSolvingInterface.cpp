/**
 * @addtogroup DFNs
 * @{
 */

#include "PerspectiveNPointSolvingInterface.hpp"

namespace CDFF
{
namespace DFN
{

PerspectiveNPointSolvingInterface::PerspectiveNPointSolvingInterface() :
inPoints(),
inProjections(),
outCamera(),
outSuccess()
{
}

PerspectiveNPointSolvingInterface::~PerspectiveNPointSolvingInterface()
{
}

void PerspectiveNPointSolvingInterface::pointsInput(const asn1SccPointcloud& data)
{
    inPoints = data;
}

void PerspectiveNPointSolvingInterface::projectionsInput(const asn1SccVisualPointFeatureVector2D& data)
{
    inProjections = data;
}

const asn1SccPose& PerspectiveNPointSolvingInterface::cameraOutput() const
{
    return outCamera;
}

bool PerspectiveNPointSolvingInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
