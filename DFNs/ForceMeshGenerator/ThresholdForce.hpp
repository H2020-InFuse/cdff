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

#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace ForceMeshGenerator
{
	/**
	 * The ThresholdForce DFN is used to build up a point cloud of an object
	 * by measuring the contact force at each point. If the force is higher
	 * than `threshold` then we consider that the arm is in contact with the
	 * surface of the object.
	 *
	 * The output of the DFN is the list of points for which we were in contact
	 * with the surface transformed into the same frame as the robot base. This
	 * puts all the measured points into the same reference system regardless of
	 * whether the robot has moved or not.
	 */

	class ThresholdForce : public ForceMeshGeneratorInterface
	{
		public:

			ThresholdForce();
            ~ThresholdForce() override = default;

			void configure() override;
			void process() override;

		private:

			struct ThresholdForceOptionsSet
			{
				double threshold;	//	Minimum force required to consider that the arm is in contact with a rigid object
			};

			Helpers::ParametersListHelper parametersHelper;
            ThresholdForceOptionsSet parameters;
            static const ThresholdForceOptionsSet DEFAULT_PARAMETERS;

			void ValidateParameters();
    };
}
}
}

#endif // FORCEMESHGENERATOR_THRESHOLDFORCE_HPP

/** @} */
