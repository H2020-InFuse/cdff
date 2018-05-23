#include "NormalMap2DInterface.hpp"
#include "Errors/Assert.hpp"


namespace dfn_ci {

using namespace FrameWrapper;

NormalMap2DInterface::NormalMap2DInterface()
{
}

NormalMap2DInterface::~NormalMap2DInterface()
{
}

void NormalMap2DInterface::imageInput(FrameConstPtr data) {
    inImage = data;
}

FrameConstPtr NormalMap2DInterface::NormalMapOutput() {
    return outNormalMap;
}

}