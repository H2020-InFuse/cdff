/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictor.cpp
 * @date 08/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 * KalmanCorrector Implementation .
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */


#include "KalmanCorrector.hpp"
#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <stdlib.h>
#include <fstream>
#include <math.h>

using namespace Helpers;

namespace dfn_ci {

KalmanCorrector::KalmanCorrector():KF(12,6,0)
	{
	parametersHelper.AddParameter<float>("KalmanParameters", "MeasurementNoiseStandardDeviationOrientation", parameters.kalmanParameters.stdOrientation, DEFAULT_PARAMETERS.kalmanParameters.stdOrientation);
	parametersHelper.AddParameter<float>("KalmanParameters", "MeasurementNoiseStandardDeviationTranslation", parameters.kalmanParameters.stdTranslation, DEFAULT_PARAMETERS.kalmanParameters.stdOrientation);

	configurationFilePath = "";
	}

KalmanCorrector::~KalmanCorrector()
{

}

void KalmanCorrector::configure() 
{
    parametersHelper.ReadFile(configurationFilePath);
    ValidateParameters();
}

void KalmanCorrector::process()
 {
       for(int i=0;i<3;i++)
	{		
       		KF.statePre.at<float>(i,0)=inPredictedState.orient.arr[i];
		KF.statePre.at<float>(i+3,0)=inPredictedState.pos.arr[i];
		KF.statePre.at<float>(i+6,0)=inPredictedState.angular_velocity.arr[i];
		KF.statePre.at<float>(i+9,0)=inPredictedState.velocity.arr[i];
	}

	ValidateInputs(KF.statePre);

	KF.errorCovPre.setTo(cv::Scalar(0));
	
	for(int row=0;row<3;row++)
	{
	  for(int col=0;col<3;col++)
	  {

		KF.errorCovPre.at<float>(row,col) = inPredictedStateCovariance.cov_orientation.arr[row].arr[col];
		KF.errorCovPre.at<float>(row+3,col+3) = inPredictedStateCovariance.cov_position.arr[row].arr[col];
		KF.errorCovPre.at<float>(row+6,col+6) =inPredictedStateCovariance.cov_angular_velocity.arr[row].arr[col];
		KF.errorCovPre.at<float>(row+9,col+9) =inPredictedStateCovariance.cov_velocity.arr[row].arr[col];
            }
	}

	cv::Mat measurement(6,1,CV_32F);
	for(int i=0;i<3;i++)
	{		
       		 measurement.at<float>(i,0)=inMeasurement.orient.arr[i];
		 measurement.at<float>(i+3,0)=inMeasurement.pos.arr[i];
		 
	}

	cv::Mat updatedState= correct(measurement);
	for(int i=0;i<3;i++)
	{
	 	outCorrectedState.orient.arr[i]=updatedState.at<float>(i);
		outCorrectedState.pos.arr[i]=updatedState.at<float>(i+3);
		outCorrectedState.angular_velocity.arr[i]=updatedState.at<float>(i+6);
		outCorrectedState.velocity.arr[i]=updatedState.at<float>(i+9);
	}
		
	for(int row=0;row<3;row++)
	{
		for(int col=0;col<3;col++)
		{
	 		outStateCovariance.cov_orientation.arr[row].arr[col]=KF.errorCovPost.at<float>(row,col);
			outStateCovariance.cov_position.arr[row].arr[col]=KF.errorCovPost.at<float>(row+3,col+3);
			outStateCovariance.cov_angular_velocity.arr[row].arr[col]=KF.errorCovPost.at<float>(row+6,col+6);
			outStateCovariance.cov_velocity.arr[row].arr[col]=KF.errorCovPost.at<float>(row+9,col+9);
		}
	}
}

const KalmanCorrector::KalmanCorrectorOptionsSet KalmanCorrector::DEFAULT_PARAMETERS =
	{
	.kalmanParameters =
		{
		.stdOrientation =1.0,
		.stdTranslation=1.0,
		}
	};

cv::Mat KalmanCorrector::correct(cv::Mat measurement)
	{
	cv::Mat stateCorrect;
	float stdOrientation;
	float stdTranslation;
        stdOrientation = parameters.kalmanParameters.stdOrientation;
	stdTranslation = parameters.kalmanParameters.stdTranslation;
	cv::setIdentity(KF.measurementNoiseCov,cv::Scalar(1));
	for(int i=0;i<3;i++)
	    {
		  for(int j=0;j<3;j++)
		 {
			KF.measurementNoiseCov.at<float>(i,i)=sqrt(stdOrientation);
			KF.measurementNoiseCov.at<float>(i+3,i+3)=sqrt(stdTranslation);
		}
	     }
	cv::Mat H;
	H=cv::Mat::zeros(6,12,CV_32F);
	KF.measurementMatrix=H.clone();
	for(int i=0;i<6;i++)
	{
	  KF.measurementMatrix.at<float>(i,i)=1;
	}
	stateCorrect= KF.correct(measurement);
	return stateCorrect;
	}

void KalmanCorrector::ValidateParameters()
	{
	ASSERT(parameters.kalmanParameters.stdOrientation > 0, " Kalman Corrector Configuration error: measurement covariance (orientation) greater than zero");
	ASSERT(parameters.kalmanParameters.stdTranslation > 0, " Kalman Corrector Configuration error: measurement covariance (translation) greater than zero");
	}

void KalmanCorrector::ValidateInputs(cv::Mat inputState)
	{
	ASSERT(inputState.rows == 12 && inputState.cols ==1, "Kalman Corrector: Corrector input must be 12-d column vector");
	}
}
