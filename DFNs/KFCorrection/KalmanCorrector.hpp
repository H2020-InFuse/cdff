/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef KALMANCORRECTOR_HPP
#define KALMANCORRECTOR_HPP

#include "KFCorrectionInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <yaml-cpp/yaml.h>

namespace dfn_ci
{
	/**
	 * Kalman corrector DFN (algorithm provided by OpenCV)
	 *
	 * States are local motion increments dp and the associated derivatives vel.
	 * Constant velocity model between two camera frames. For pose estimation,
	 * the user of this DFN can use corrected states, for example convert to
	 * absolute corrected pose as T_{k+1} = T_k * dT(dp_corrected)
	 *
	 * @param measurement noise parameters
	 *        standard deviation of pose estimate (rotation and translation)
	 */
	class KalmanCorrector : public KFCorrectionInterface
	{
		public:

			KalmanCorrector();
			virtual ~KalmanCorrector();

			virtual void configure();
			virtual void process();

			cv::KalmanFilter KF;

		private:

			struct KalmanParameters
			{
				float stdOrientation;
				float stdTranslation;
			};
			struct KalmanCorrectorOptionsSet
			{
				KalmanParameters kalmanParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			KalmanCorrectorOptionsSet parameters;
			static const KalmanCorrectorOptionsSet DEFAULT_PARAMETERS;

			cv::Mat correct(cv::Mat measurement);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputState);
			void Configure(const YAML::Node& configurationNode);
	};
}

#endif // KALMANCORRECTOR_HPP

/** @} */
