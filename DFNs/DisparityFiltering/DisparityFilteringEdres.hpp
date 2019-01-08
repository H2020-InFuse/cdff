/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP
#define DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP

#include "DisparityFilteringInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityFiltering
{
    /**
     * @brief Implementation of the disparity filetring algorithm provided by EDRES library
     * For open-source implementation see DisparityImage DFN.
     */
    class DisparityFilteringEdres : public DisparityFilteringInterface
    {
        public:

            DisparityFilteringEdres();
            virtual ~DisparityFilteringEdres();

            virtual void configure();
            virtual void process();

            struct DisparityFilteringEdresParams
            {
                /**
                 * @brief Trimming width in pixels
                 */
                int   trimWidth;

                /**
                 * @brief Connexity threshold
                 */
                float connexityThresh;

                /**
                 * @brief Minimum surface of areas to be kept
                 */
                int   surfMin;

                /**
                 * @brief Maximum surface of areas to be kept
                 */
                int   surfMax;
            };

            Helpers::ParametersListHelper              parametersHelper;
            DisparityFilteringEdresParams              parameters;
            static const DisparityFilteringEdresParams DEFAULT_PARAMETERS;

            void ValidateParameters();
    };
}
}
}

#endif // DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP

/** @} */
