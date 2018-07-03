/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictor.hpp
 * @date 08/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 *
 *  @brief This DFN executes a Kalman Corrector.
 *
 * This DFN is Kalman Predictor implementation of OpenCV.
 *
 * states are incremental local motions, dp and associated derivatives, vel
 * Constant velocity model, between two camera frames
 * For pose estimation, the user of this DFN can use corrected states, for example convert to corrected pose dT(dp_correct)
 * This DFN implementation requires the following parameters:
 * @param  measurement noise parameters, standard deviation of pose estimation (rotation and translation)  .
 *
 * @{
 */

#ifndef KALMANCORRECTOR_HPP
#define KALMANCORRECTOR_HPP
/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "KFCorrection/KFCorrectionInterface.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>



namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
	class KalmanCorrector: public KFCorrectionInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		KalmanCorrector();
		virtual ~KalmanCorrector();
		virtual void process();
		virtual void configure();

		cv::KalmanFilter KF;


	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
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
#endif
