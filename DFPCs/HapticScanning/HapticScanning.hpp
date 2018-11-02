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
     * TODO Class documentation
     */
    class HapticScanning : public HapticScanningInterface
    {
        public:
            HapticScanning();
            virtual ~HapticScanning();

            virtual void setup();
            virtual void run();

        private:
            DfpcConfigurator configurator;
            std::unique_ptr<CDFF::DFN::ForceMeshGeneratorInterface> m_force_mesh_generator;

            void InstantiateDFNs();
    };
}
}

#endif // HAPTICSCANNING_HPP

/** @} */
