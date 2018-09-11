/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_THRESHOLDFORCE_HPP
#define FORCEMESHGENERATOR_THRESHOLDFORCE_HPP

#include "ForceMeshGeneratorInterface.hpp"
#include <Frame.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace ForceMeshGenerator
{
	/**
	 * Construction of a point cloud with force sensor inputs
	 *
	 * @param generalParameters.threshold
	 */

	class ThresholdForce : public ForceMeshGeneratorInterface
	{
		public:

			ThresholdForce();
            virtual ~ThresholdForce() = default;

			virtual void configure();
			virtual void process();

		private:

			struct ThresholdForceOptionsSet
			{
				double threshold;
			};

			Helpers::ParametersListHelper parametersHelper;
            ThresholdForceOptionsSet parameters;
            static const ThresholdForceOptionsSet DEFAULT_PARAMETERS;


			Converters::PclPointCloudToPointCloudConverter pclPointCloudToPointCloud;

            pcl::PointCloud<pcl::PointXYZ>::Ptr CreatePointCloud();

			void ValidateParameters();
			void ValidateInputs();
    };
}
}
}

#endif // FORCEMESHGENERATOR_THRESHOLDFORCE_HPP

/** @} */
