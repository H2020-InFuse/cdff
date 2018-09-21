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
            virtual ~HapticScanning();

            virtual void setup();
            virtual void run();

        private:
            DfpcConfigurator configurator;
            std::unique_ptr<CDFF::DFN::ForceMeshGeneratorExecutor> m_force_mesh_generator;

            void InstantiateDFNExecutors();
    };
}
}

#endif // HAPTICSCANNING_HPP

/** @} */
