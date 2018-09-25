/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR



****************************************************************************//**

 \file KalmanFilter.h
 
*****************************************************************************/
#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

class KalmanFilter
{
	public:

		KalmanFilter();

		int setParams(double sig_acc_rot, double sig_acc_trasl, double initcov);
		void init(double * T0, double * vel0, double time0);

		int predict(double time_curr);

		int correct(double * T_meas, double * cov_meas, double time_curr);

		
		void dumpValues(void);

		double m_A[12*12];
		double m_AT[12*12];
		double m_sigma_acc_rot;
		double m_sigma_acc_trasl;
		double m_W[12*12];
		double m_H[6*12];
		double m_HT[12*6];
		double m_init_cov;
		double m_Tpred[16];
		double m_state_pred[12];
		double m_cov_pred[12*12];
		double m_cov_tmp[12*12];
		double m_cov_tmp2[12*12];
		double m_cov_tmp3[12*12];
		double m_cov_UT[6*6];
		double m_cov_V[6*6];
		double m_K[12*6];
		double m_KT[6*12];
		double m_Test[16];
		double m_state_est[12];
		double m_cov_est[12*12];
		double m_time_est;
		bool m_use_AngleAxis;
};

#endif /* KALMANFILTER_H_ */
