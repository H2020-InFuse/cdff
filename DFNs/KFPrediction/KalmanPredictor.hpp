/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictor.hpp
 * @date 24/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN executes a Kalman Predictor.
 *  
 * This DFN is Kalman Predictor implementation of OpenCV.
 * 
 * states are incremental local motions dp and associated derivatives vel
 * Constant velocity model, between two camera frames  
 * For pose estimation, the user of this DFN can use predicted states, for example convert to absolute predicted pose as T_k+1=T_k*dT(dp)
 * This DFN implementation requires the following parameters:
 * @param  porcess noise parameters, standard deviation of the random acc (linear and angualr)  .
 * @param, initial covariance, roto-translation and velocity components .
 *
 * @{
 */

#ifndef KALMANPREDICTOR_HPP
#define KALMANPREDICTOR_HPP
/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <KFPrediction/KFPredictionInterface.hpp>
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
    class KalmanPredictor : public KFPredictionInterface
    {
        public:
            KalmanPredictor();
            virtual ~KalmanPredictor();
            virtual void process();
            virtual void configure();
	   
	    float timeOfLastValidCorrection;
	    cv::KalmanFilter KF; 
	    void init(cv::Mat &pose,cv::Mat &vel0,float time0);
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
#endif
