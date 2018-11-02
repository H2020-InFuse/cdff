/**
 * @addtogroup DFPCs
 * @{
 */

#include "HapticScanning.hpp"

#include <Executors/ForceMeshGenerator/ForceMeshGeneratorExecutor.hpp>

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
    InstantiateDFNs();
}

void HapticScanning::run()
{
    CDFF::DFN::Executors::Execute(m_force_mesh_generator.get(), inArmBasePose, inArmEndEffectorPose, inArmEndEffectorWrench, outPointCloud);
}

void HapticScanning::InstantiateDFNs()
{
    m_force_mesh_generator.reset( dynamic_cast<CDFF::DFN::ForceMeshGeneratorInterface*>( configurator.GetDfn("forceMeshGenerator") ) );
}

}
}

/** @} */
