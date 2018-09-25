/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef HAPTICSCANNING_HPP
#define HAPTICSCANNING_HPP

#include "HapticScanningInterface.hpp"
#include <DfpcConfigurator.hpp>
#include <ForceMeshGenerator/ForceMeshGeneratorExecutor.hpp>

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
            ~HapticScanning() override;

            void setup() override;
            void run() override;

        private:
            DfpcConfigurator configurator;
            std::unique_ptr<CDFF::DFN::ForceMeshGeneratorExecutor> m_force_mesh_generator;

            void InstantiateDFNExecutors();
    };
}
}

#endif // HAPTICSCANNING_HPP

/** @} */
