/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

 ****************************************************************************//**

 \file SimplePredictor.h
 

*****************************************************************************/
#ifndef SIMPLEPREDICTOR_H_
#define SIMPLEPREDICTOR_H_

namespace DLRtracker
{
class SimplePredictor
{
	public:

		SimplePredictor();

		void init(double * T0, double * vel0, double time0);

		int predict(double time_curr);

		int correct(double * T_meas, double time_curr);

		double m_A[12*12];
		
		double m_Tpred[16];
		double m_state_pred[12];
		
		double m_Test[16];
		double m_state_est[12];
		
		double m_time_est;
		
		bool m_use_AngleAxis;
};

}

#endif /* SIMPLEPREDICTOR_H_ */
