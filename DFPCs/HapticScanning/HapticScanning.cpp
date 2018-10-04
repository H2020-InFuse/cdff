/**
 * @addtogroup DFPCs
 * @{
 */

#include "HapticScanning.hpp"

namespace CDFF
{
namespace DFPC
{

HapticScanning::HapticScanning()
: m_force_mesh_generator( nullptr )
{
    configurationFilePath = "";
}

HapticScanning::~HapticScanning()
{
}

void HapticScanning::setup()
{
    configurator.configure(configurationFilePath);
    InstantiateDFNExecutors();
}

void HapticScanning::run()
{
    m_force_mesh_generator->Execute(inArmBasePose, inArmEndEffectorPose, inArmEndEffectorWrench, outPointCloud);
}

void HapticScanning::InstantiateDFNExecutors()
{
    m_force_mesh_generator.reset( new CDFF::DFN::ForceMeshGeneratorExecutor( dynamic_cast<CDFF::DFN::ForceMeshGeneratorInterface*>( configurator.GetDfn("forceMeshGenerator") ) ) );
}

}
}

/** @} */
