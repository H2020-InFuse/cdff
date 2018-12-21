/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "KalmanPredictor.hpp"

#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Helpers;

namespace CDFF
{
namespace DFN
{
namespace KFPrediction
{

KalmanPredictor::KalmanPredictor()
	: timeOfLastValidCorrection(-1),KF(12,6,0),Tpred(4,4,CV_32F),Test(4,4,CV_32F)
{
        parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("KalmanParameters", "StandardDeviationtdOrientation", parameters.kalmanParameters.stdOrientation, DEFAULT_PARAMETERS.kalmanParameters.stdOrientation);
	parametersHelper.AddParameter<float>("KalmanParameters", "StandardDeviationtdTranslation", parameters.kalmanParameters.stdTranslation, DEFAULT_PARAMETERS.kalmanParameters.stdOrientation);
	parametersHelper.AddParameter<float>("KalmanParameters", "InitialStateCovariance", parameters.kalmanParameters.initialCovariance, DEFAULT_PARAMETERS.kalmanParameters.initialCovariance);

	configurationFilePath = "";
}

KalmanPredictor::~KalmanPredictor()
{
}

void KalmanPredictor::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void KalmanPredictor::process()
{

	cv::Mat pose(6,1,CV_32F);
	cv::Mat vel0(6,1,CV_32F);
	float time0 = 0;   // pose initialization timestamp
	float currentTime; // image timestamp

	// Read data from input port
	for (int i = 0; i < 3; i++)
	{
		pose.at<float>(i,0) = inPreviousState.orient.arr[i];
		pose.at<float>(i+3,0) = inPreviousState.pos.arr[i];
		vel0.at<float>(i,0) = inPreviousState.angular_velocity.arr[i];
		vel0.at<float>(i+3,0) = inPreviousState.velocity.arr[i];
	}

	currentTime = (float)inPreviousState.timestamp.microseconds;

	// Process data
	ValidateInputs(pose);

	if (time0 == 0) // DFN user should decide when to initialize
	{
		init(pose,vel0,time0);
	}

	cv::Mat predictor = predict(currentTime);

	// Write data to output port
	for (int i = 0; i < 3; i++)
	{
		outPredictedState.orient.arr[i] = predictor.at<float>(i);
		outPredictedState.pos.arr[i] = predictor.at<float>(i+3);
		outPredictedState.angular_velocity.arr[i] = predictor.at<float>(i+6);
		outPredictedState.velocity.arr[i] = predictor.at<float>(i+9);
	}

	cv::Mat covPredictor = predict();

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			outPredictedStateCovariance.cov_orientation.arr[row].arr[col] = covPredictor.at<float>(row,col);
			outPredictedStateCovariance.cov_position.arr[row].arr[col] = covPredictor.at<float>(row+3,col+3);
			outPredictedStateCovariance.cov_angular_velocity.arr[row].arr[col] = covPredictor.at<float>(row+6,col+6);
			outPredictedStateCovariance.cov_velocity.arr[row].arr[col] = covPredictor.at<float>(row+9,col+9);
		}
	}
}

const KalmanPredictor::KalmanPredictorOptionsSet KalmanPredictor::DEFAULT_PARAMETERS =
{
	//.kalmanParameters =
	{
		/*.stdOrientation =*/ 1000.0,
		/*.stdTranslation =*/ 1000.0,
		/*.initialCovariance =*/ 10000
	}
};

cv::Mat KalmanPredictor::predict(float currentTime)
{
	cv::Mat statePredict;
	float stdOrientation, stdTranslation;
	stdOrientation = parameters.kalmanParameters.stdOrientation;
	stdTranslation = parameters.kalmanParameters.stdTranslation;
	float dt = currentTime-timeOfLastValidCorrection;

	cv::Mat G_rot_temp1(3,6,CV_32F,cv::Scalar(0));
	cv::Mat G_rot_temp2(3,6,CV_32F,cv::Scalar(0));

	cv::Mat G_trans_temp1(3,6,CV_32F,cv::Scalar(0));
	cv::Mat G_trans_temp2(3,6,CV_32F,cv::Scalar(0));

	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			G_rot_temp1.at<float>(i,i)=0.5*dt*dt*stdOrientation;
			G_rot_temp2.at<float>(i,i+3)=dt*stdOrientation;

			G_trans_temp1.at<float>(i,i)=0.5*dt*dt*stdTranslation;
			G_trans_temp2.at<float>(i,i+3)=dt*stdTranslation;
		}
	}

	cv::Mat G1(6,6,CV_32F);
	cv::vconcat(G_rot_temp1,G_rot_temp2, G1);

	cv::Mat G2(6,6,CV_32F);
	cv::vconcat(G_trans_temp1,G_trans_temp2, G2);

	cv::Mat G(12,6,CV_32F);
	cv::vconcat(G1,G2, G);

	cv::Mat Gt(6,12,CV_32F);
	Gt= G.t();

	KF.processNoiseCov =G*Gt;

	setIdentity(KF.transitionMatrix, cv::Scalar::all(1));
	KF.transitionMatrix.at<float>(0,6)=dt;
	KF.transitionMatrix.at<float>(1,7)=dt;
	KF.transitionMatrix.at<float>(2,8)=dt;
	KF.transitionMatrix.at<float>(3,9)=dt;
	KF.transitionMatrix.at<float>(4,10)=dt;
	KF.transitionMatrix.at<float>(5,11)=dt;

	statePredict=KF.predict();

	return statePredict;
}

cv::Mat KalmanPredictor::predict()
{
	cv::Mat A=KF.transitionMatrix;
	cv::Mat Q=KF.processNoiseCov;
	KF.errorCovPre=A*KF.errorCovPost*A+Q;

	return KF.errorCovPre;
}

void KalmanPredictor::init(cv::Mat &pose, cv::Mat &vel0, float time0)
{

	for (int i=0;i<6;i++)
	{
		KF.statePost.at<float>(i)= 0;
		KF.statePost.at<float>(i+6)=vel0.at<float>(i,0);
	}

	float initialCovariance;
	initialCovariance = parameters.kalmanParameters.initialCovariance;
	setIdentity(KF.errorCovPost, cv::Scalar::all(initialCovariance));

	timeOfLastValidCorrection = time0;
}

void KalmanPredictor::ValidateParameters()
{
	ASSERT(parameters.kalmanParameters.initialCovariance > 100, " Kalman Predictor Configuration error: initial state covariance too small");
}

void KalmanPredictor::ValidateInputs(cv::Mat pose)
{
	ASSERT(pose.rows == 6 && pose.cols ==1, "Kalman Predictor: Predictor input is 6-d column vector");
}

}
}
}

/** @} */
