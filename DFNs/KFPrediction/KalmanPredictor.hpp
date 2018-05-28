/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef KALMANPREDICTOR_HPP
#define KALMANPREDICTOR_HPP

#include "KFPredictionInterface.hpp"

#include <Helpers/ParametersListHelper.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace dfn_ci
{
	/**
	 * Kalman predictor DFN (algorithm provided by OpenCV)
	 *
	 * States are local motion increments dp and the associated derivatives vel.
	 * Constant velocity model between two camera frames. For pose estimation,
	 * the user of this DFN can use predicted states, for example convert to
	 * absolute predicted pose as T_{k+1} = T_k * dT(dp)
	 *
	 * @param  process noise parameters
	 *         standard deviation of the random (linear and angular) acceleration
	 * @param initial covariance
	 *        roto-translation and velocity components
	 */
	class KalmanPredictor : public KFPredictionInterface
	{
		public:

			KalmanPredictor();
			virtual ~KalmanPredictor();

			virtual void configure();
			virtual void process();

			float timeOfLastValidCorrection;
			cv::KalmanFilter KF;
			void init(cv::Mat &pose, cv::Mat &vel0, float time0);
			cv::Mat Tpred;
			cv::Mat Test;

		private:

			struct KalmanParameters
			{
				float stdOrientation;
				float stdTranslation;
				float initialCovariance;
			};

			struct KalmanPredictorOptionsSet
			{
				KalmanParameters kalmanParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			KalmanPredictorOptionsSet parameters;

			static const KalmanPredictorOptionsSet DEFAULT_PARAMETERS;

			cv::Mat predict(float currentTime);
			cv::Mat predict();

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputState);

			void Configure(const YAML::Node& configurationNode);
	};
}

#endif // KALMANPREDICTOR_HPP

/** @} */
