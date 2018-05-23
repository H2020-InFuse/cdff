#include "KMeans2DInterface.hpp"
#include "Errors/Assert.hpp"


namespace dfn_ci {

using namespace FrameWrapper;

KMeans2DInterface::KMeans2DInterface()
{
}

KMeans2DInterface::~KMeans2DInterface()
{
}

void KMeans2DInterface::imageInput(FrameConstPtr data) {
    inImage = data;
}

FrameConstPtr KMeans2DInterface::KMeansImageOutput() {
    return outKMeansImage;
}

}