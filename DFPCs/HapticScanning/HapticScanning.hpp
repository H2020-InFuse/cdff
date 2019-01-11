/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef HAPTICSCANNING_HPP
#define HAPTICSCANNING_HPP

#include "HapticScanningInterface.hpp"
#include <DfpcConfigurator.hpp>
#include <ForceMeshGenerator/ForceMeshGeneratorInterface.hpp>

namespace CDFF
{
namespace DFPC
{
    /**
     * The Haptic Scanning DFPC is used to build up a point cloud of an object
     * by touching it with a robotic arm and measuring the contact force at each point.
     *
     * The output of the DFPC is the list of points for which the arm was in contact
     * with the object.
     */

    class HapticScanning : public HapticScanningInterface
    {
        public:
            HapticScanning();
            virtual ~HapticScanning();

            virtual void setup() override;
            virtual void run() override; 

        private:
            DfpcConfigurator configurator;
            CDFF::DFN::ForceMeshGeneratorInterface* m_force_mesh_generator;

            void InstantiateDFNs();
    };
}
}

#endif // HAPTICSCANNING_HPP

/** @} */
