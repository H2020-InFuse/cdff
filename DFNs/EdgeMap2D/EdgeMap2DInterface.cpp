#include "EdgeMap2DInterface.hpp"
#include "Errors/Assert.hpp"


namespace dfn_ci {

using namespace FrameWrapper;

EdgeMap2DInterface::EdgeMap2DInterface()
{
}

EdgeMap2DInterface::~EdgeMap2DInterface()
{
}

void EdgeMap2DInterface::imageInput(FrameConstPtr data) {
    inImage = data;
}

FrameConstPtr EdgeMap2DInterface::EdgeMapOutput() {
    return outEdgeMap;
}

}