#ifndef EDGETRACKERHELPER_HPP
#define EDGETRACKERHELPER_HPP

#include <Errors/Assert.hpp>

#include <cmath>    // std::sqrt std::cos std::sin std::acos std::fmax std::fabs
#include <cstring>  // std::memset std::memcpy
#include <iostream> // std::cout std::endl

namespace EdgeTrackerHelper
{

void matrixTranspose(const double * A, double * AT, const int m, const int n)
{
	ASSERT(A != AT,
		"ERROR: matrixTranspose not implemented for in-place operation!");

	int i, j;
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j < m; ++j)
		{
			AT[m*i+j] = A[n*j+i];
		}
	}
}

void matrixProduct333(const double * A, const double * B, double * C)
{
	ASSERT((B != C) || (A != C),
		"ERROR: matrixProduct333 not implemented for in-place operation!");

	C[0] = A[0]*B[0]+A[1]*B[3]+A[2]*B[6];
	C[1] = A[0]*B[1]+A[1]*B[4]+A[2]*B[7];
	C[2] = A[0]*B[2]+A[1]*B[5]+A[2]*B[8];

	C[3] = A[3]*B[0]+A[4]*B[3]+A[5]*B[6];
	C[4] = A[3]*B[1]+A[4]*B[4]+A[5]*B[7];
	C[5] = A[3]*B[2]+A[4]*B[5]+A[5]*B[8];

	C[6] = A[6]*B[0]+A[7]*B[3]+A[8]*B[6];
	C[7] = A[6]*B[1]+A[7]*B[4]+A[8]*B[7];
	C[8] = A[6]*B[2]+A[7]*B[5]+A[8]*B[8];
}

void matrixProduct444(const double * A, const double * B, double * C)
{
	ASSERT((B != C) || (A != C),
		"ERROR: matrixProduct444 not implemented for in-place operation!");

	C[0] = A[0]*B[0]+A[1]*B[4]+A[2]*B[8]+A[3]*B[12];
	C[1] = A[0]*B[1]+A[1]*B[5]+A[2]*B[9]+A[3]*B[13];
	C[2] = A[0]*B[2]+A[1]*B[6]+A[2]*B[10]+A[3]*B[14];
	C[3] = A[0]*B[3]+A[1]*B[7]+A[2]*B[11]+A[3]*B[15];

	C[4] = A[4]*B[0]+A[5]*B[4]+A[6]*B[8]+A[7]*B[12];
	C[5] = A[4]*B[1]+A[5]*B[5]+A[6]*B[9]+A[7]*B[13];
	C[6] = A[4]*B[2]+A[5]*B[6]+A[6]*B[10]+A[7]*B[14];
	C[7] = A[4]*B[3]+A[5]*B[7]+A[6]*B[11]+A[7]*B[15];

	C[8] = A[8]*B[0]+A[9]*B[4]+A[10]*B[8]+A[11]*B[12];
	C[9] = A[8]*B[1]+A[9]*B[5]+A[10]*B[9]+A[11]*B[13];
	C[10] = A[8]*B[2]+A[9]*B[6]+A[10]*B[10]+A[11]*B[14];
	C[11] = A[8]*B[3]+A[9]*B[7]+A[10]*B[11]+A[11]*B[15];

	C[12] = A[12]*B[0]+A[13]*B[4]+A[14]*B[8]+A[15]*B[12];
	C[13] = A[12]*B[1]+A[13]*B[5]+A[14]*B[9]+A[15]*B[13];
	C[14] = A[12]*B[2]+A[13]*B[6]+A[14]*B[10]+A[15]*B[14];
	C[15] = A[12]*B[3]+A[13]*B[7]+A[14]*B[11]+A[15]*B[15];
}

void rotTranslFromT(const double * T, double * R, double * t)
{
	R[0] = T[0];
	R[1] = T[1];
	R[2] = T[2];
	t[0] = T[3];
	R[3] = T[4];
	R[4] = T[5];
	R[5] = T[6];
	t[1] = T[7];
	R[6] = T[8];
	R[7] = T[9];
	R[8] = T[10];
	t[2] = T[11];
}

int matrixRmatToRvec(const double * Rmat, double * rvec)
{
	double R[9], rx, ry, rz;
	double theta, s, c;

	std::memcpy(R, Rmat, 9*sizeof(double));

	rx = R[7] - R[5];
	ry = R[2] - R[6];
	rz = R[3] - R[1];

	s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
	c = (R[0] + R[4] + R[8] - 1)*0.5;
	c = c > 1.0 ? 1.0 : c < -1.0 ? -1.0 : c;
	theta = std::acos(c);

	if (s < 1e-5)
	{
		double t;

		if (c > 0)
		{
			rx = ry = rz = 0;
		}
		else
		{
			t = (R[0] + 1)*0.5;
			rx = std::sqrt(std::fmax(t,0.0));
			t = (R[4] + 1)*0.5;
			ry = std::sqrt(std::fmax(t,0.0))*(R[1] < 0 ? -1.0 : 1.0);
			t = (R[8] + 1)*0.5;
			rz = std::sqrt(std::fmax(t,0.0))*(R[2] < 0 ? -1.0 : 1.0);
			if (std::fabs(rx) < std::fabs(ry) && std::fabs(rx) < std::fabs(rz)
				&& (R[5] > 0) != (ry*rz > 0))
			{
				rz = -rz;
			}
			theta /= std::sqrt(rx*rx + ry*ry + rz*rz);
			rx *= theta;
			ry *= theta;
			rz *= theta;
		}
	}
	else
	{
		double vth = 1.0/(2*s);

		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}

	rvec[0] = rx;
	rvec[1] = ry;
	rvec[2] = rz;

	return 0;
}

int AngleAxisFromT(const double * T, double * rotTrasl)
{
	double R[9];
	double t[3], rho[3];

	// Extract R,t
	rotTranslFromT(T, R, t);

	// Rodrigues -> rho
	int err_code = matrixRmatToRvec(R, rho);
	if (err_code != 0)
	{
		return err_code;
	}

	// Convert output to degrees
	rotTrasl[0] = rho[0]*180.0/M_PI;
	rotTrasl[1] = rho[1]*180.0/M_PI;
	rotTrasl[2] = rho[2]*180.0/M_PI;

	rotTrasl[3] = t[0];
	rotTrasl[4] = t[1];
	rotTrasl[5] = t[2];

	return 0;
}

void matrixIdentity(double * A, int m)
{
	std::memset(A, 0, m*m*sizeof(double));

	for (int i = 0; i < m; i++)
	{
		A[i*(m+1)] = 1;
	}
}

void matrixRvecToRmat(const double * rvec, double * Rmat)
{
	double rx, ry, rz, theta;

	rx = rvec[0];
	ry = rvec[1];
	rz = rvec[2];

	theta = std::sqrt(rx*rx + ry*ry + rz*rz);

	if (theta < DBL_EPSILON)
	{
		matrixIdentity(Rmat, 3);
	}
	else
	{
		const double I[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

		double c = std::cos(theta);
		double s = std::sin(theta);
		double c1 = 1.0 - c;
		double itheta = theta ? 1.0/theta : 0.0;

		rx *= itheta; ry *= itheta; rz *= itheta;

		double rrt[] =
			{rx*rx, rx*ry, rx*rz, rx*ry, ry*ry, ry*rz, rx*rz, ry*rz, rz*rz};
		double _r_x_[] =
			{0, -rz, ry, rz, 0, -rx, -ry, rx, 0};

		for (int k = 0; k < 9; k++)
		{
			Rmat[k] = c*I[k] + c1*rrt[k] + s*_r_x_[k];
		}
	}
}

void printMatrix(const char * title, const double * mat, const int m, const int n)
{
	std::cout << title << " = [";
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			std::cout << mat[n*i+j] << " ";
		}
		if (i < m-1)
		{
			std::cout << ";...\n";
		}
		else
		{
			std::cout << "];" << std::endl;
		}
	}
}

};

#endif // EDGETRACKERHELPER_HPP
