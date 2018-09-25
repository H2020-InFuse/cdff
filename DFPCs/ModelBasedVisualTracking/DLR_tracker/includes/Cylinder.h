/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR
 
****************************************************************************//**

 \file Cylinder.h
 
*****************************************************************************/
#ifndef CYLINDERMODEL_H_
#define CYLINDERMODEL_H_

#include "common.h"

class Cylinder
{
	public:

		void init(double rc, double ncut_plane1[3], double Ccut_plane1[3], double ncut_plane2[3], double Ccut_plane2[3]);
		
		void changeLocalFrame(double * dT);

		double m_r_obj[3];

		double m_Q_obj[3];

		double m_cut_plane1[4];
		double m_cut_plane2[4];
		
		double m_rc;
};

#endif /* CYLINDERMODEL_H_ */
