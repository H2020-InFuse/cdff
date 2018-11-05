/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

 

******************************************************************************

 \file common.h

*****************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_


#define _USE_OPENCV_DISPLAY
#define _USE_STD
#define _DO_BIG_ALLOCATION
#ifdef _USE_STD
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <stddef.h>
#endif

#include <string.h>
#include <cstring>
#include <math.h>

#ifndef DBL_EPSILON
#define DBL_EPSILON 2.2204460492503131e-16
#endif

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#define _MAX_NCAMS 5
#define _MAX_NBODIES 50
#define _MAX_NROIS 50
#define _MAX_NV_ROI 10
#define _MAX_NV_POLYGON 100
#define _MAX_PARSE_ARRAY 300000
#define _MAX_LENGTH_GLOBAL_STRING 1000000
#define _MAX_NLOCAL_FRAMES 100

#define _FAILURE_INTERNAL -1
#define _FAILURE_POSEEST_OUTLIERS -2
#define _FAILURE_VLD -3
#define _FAILURE_POSEEST_DIVERGENCE -4
#define _FAILURE_POSEEST_OUT_OF_SCREEN -5
#define _FAILURE_POSEEST_MAX_ITER -6
#define _FAILURE_SVD -7
#define _FAILURE_QSORT -8
#define _FAILURE_POSEEST_REPROJ_ERROR -9
#define _FAILURE_PARSE_CAMERA -10
#define _FAILURE_PARSE_TRACKER -11
#define _FAILURE_PARSE_MODEL -12
#define _FAILURE_PARSE_POLYHEDRON -13
#define _FAILURE_PARSE_POLYLINE -14
#define _FAILURE_PROJECT_POINT -15
#define _FAILURE_CANNY -16


#define SQR(x) ((x)*(x))
#define SWAP(a,b,temp) temp=(a);(a)=(b);(b)=temp;
//#define MAX(a,b) ((a) > (b) ? (a) : (b))
//#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x)<0 ? -(x) : (x))
#define SGN(a)	(((a)<0) ? -1 : 1)

#define MY_NEXT_LINE_POINT( line_iterator )                     \
{                                                               \
    int _line_iterator_mask = (line_iterator).err < 0 ? -1 : 0; \
    (line_iterator).err += (line_iterator).minus_delta +        \
        ((line_iterator).plus_delta & _line_iterator_mask);     \
    (line_iterator).ptr += (line_iterator).minus_step +         \
        ((line_iterator).plus_step & _line_iterator_mask);      \
}

#if defined WIN32 || defined _WIN32 || defined WINCE
inline double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}
#endif

typedef struct myLineIterator
{
    unsigned char* ptr;
    int  err;
    int  plus_delta;
    int  minus_delta;
    int  plus_step;
    int  minus_step;
}
myLineIterator;

typedef struct
{
	double val;
	int ind;
} qsort_type;

namespace DLRtracker
{
	extern unsigned int m_global_output_string_counter;
	extern char m_global_output_string[_MAX_LENGTH_GLOBAL_STRING];

	extern size_t m_currentAllocUchar;
	extern size_t m_currentAllocUcharPtr;
	extern size_t m_currentAllocDouble;
	extern size_t m_currentAllocFloat;
	extern size_t m_currentAllocShort;
	extern size_t m_currentAllocInt;
	extern size_t m_currentAllocIntPtr;

#ifdef _DO_BIG_ALLOCATION
	extern size_t m_totalAllocUchar;
	extern size_t m_totalAllocUcharPtr;
	extern size_t m_totalAllocDouble;
	extern size_t m_totalAllocFloat;
	extern size_t m_totalAllocShort;
	extern size_t m_totalAllocInt;
	extern size_t m_totalAllocIntPtr;

	extern unsigned char* m_bigArrayUchar;
	extern unsigned char** m_bigArrayUcharPtr;
	extern double* m_bigArrayDouble;
	extern float* m_bigArrayFloat;
	extern short* m_bigArrayShort;
	extern int* m_bigArrayInt;
	extern int** m_bigArrayIntPtr;

	void doBigMalloc(size_t nUchar, size_t nUcharPtr, size_t nDouble, size_t nFloat, size_t nInt, size_t nIntPtr, size_t nShort);
	void doBigFree(void);
#endif
	
	enum classTags {CLASS_POLYHEDRON = 0, CLASS_POLYLINE = 1, CLASS_CYLINDER = 2, INVALID_OBJECT_CLASS = _FAILURE_INTERNAL};

	unsigned char* myMallocUchar(size_t n);
	unsigned char** myMallocUcharPtr(size_t n);
	double* myMallocDouble(size_t n);
	float* myMallocFloat(size_t n);
	int* myMallocInt(size_t n);
	int** myMallocIntPtr(size_t n);
	short* myMallocShort(size_t n);

	void myFree (void* ptr);
	void resetMemoryCounters(void);

	void dumpMemoryAlloc(void);

	int myQuickSort(double *arr, int *indices, int n);

	double myMOD(double x, double y);

	bool clipLine2D( const int w, const int h, int pt1[2], int pt2[2] );

	int clipSegmentToViewingFrustum(const double * segment_in, double * segments_out, double * planes_frustum, double min_length);

	int initLineIterator(unsigned char *img, const int w, const int h, const int c, const int pt1[2], const int pt2[2], myLineIterator * iterator, const int connectivity, const int left_to_right);

	void convertDoubleToChar(double * src, unsigned char * dst, int w, int h, double scale, double shift);

	void convertShortToChar(short * src, unsigned char * dst, int w, int h, double scale, double shift);

	void convertColorToGray(const unsigned char * imgc, unsigned char * img, const int w, const int h);

	void convertGrayToColor(const unsigned char * img, unsigned char * imgc, const int w, const int h);

	void merge(const unsigned char * ch1, const unsigned char * ch2, const unsigned char * ch3, unsigned char * out, const int w, const int h);

	void initUndistortRectifyMap( const double * K, const double * distCoeffs, double * matR, double * newK, const int width, const int height, double * map1, double * map2 );

	void initUndistortionTables(const double * map1, const double * map2, int * w1_table, int * w2_table, int * w3_table, int * w4_table, int * offset_table, const int width, const int height);

	void undistortImageFromTables(const unsigned char * src, unsigned char * dst, const int * w1_table, const int * w2_table, const int * w3_table, const int * w4_table, const int * offset_table, const int width, const int height);

	int CannyAndSobel(unsigned char * src, short * dx, short * dy, unsigned char * dst, int * canny_buffer, unsigned char ** canny_stack, short * sobel_buffer, int w, int h, int thresh1, int thresh2);

	int myCanny( short * dx, short * dy, unsigned char * dst, int * canny_buffer, unsigned char ** stack, int w, int h, double low_thresh, double high_thresh);

	int mySobel( unsigned char * src, short * dx, short * dy, short * tmp_buffer, int width, int height);

	void globalStrcat(const char * str, int nchar);

	void dumpGlobalStrToFile(const char * fname);

	long int getTickCount(void);

	long int getTickFrequency(void);

	double showElapsed(const char * title, long int tic, long int toc);

	void createWindow(const std::string& title, int resize);

	void showImage(const std::string& title, unsigned char * img, int w, int h, int c);

	void drawLine(unsigned char * image, const int xres, const int yres, bool isColor, const int pt1[2], const int pt2[2], const unsigned char color[3]);

	void drawRect(unsigned char * image, const int xres, const int yres, bool isColor, const int pt1[2], const int pt2[2], const unsigned char color[3]);

	void drawCircle(unsigned char * img, const int width, const int height, const bool isColor, const int center[2], const int radius, unsigned char color[3], const bool fill = false);

	void drawMatchingPoints(int npt, const double * reproj, const double * reproj_normals, const double * match, const double * weights, unsigned char * img, const int xres, const int yres, bool draw_normals, bool draw_matching, double search_dist);
	void drawCartesianFrame(const double * K, const double * T, const double axis_length, unsigned char * img, const int w, const int h, const bool isColor);

	char waitKey(int time);

	void destroyWindow(const std::string& title);

	void destroyAllWindows(void);
	
	int matrixSVD(double * A, int m, int n, double * S, double * U, double * V, int transposeU, int transposeV, double * workd, int * worki);
	
	void matrixProduct(const double * A, const double * B, double * C, const int M, const int Q, const int N);
	
	void matrixTranspose(const double * A, double * AT, const int m, const int n);

	void imageCopy(const unsigned char * A, unsigned char * B, int w, int h);

	void matrixCopy(const double * A, double * B, int m, int n);

	void matrixZero(double * A, int m, int n);

	void matrixIdentity(double * A, int m);

	void matrixInverseK(const double * K, double * KI);

	void matrixProduct333(const double * A, const double * B, double * C);

	void matrixProduct344(const double * A, const double * B, double * C);

	void matrixProduct444(const double * A, const double * B, double * C);

	void matrixTranspose33(const double * A, double * AT);

	void matrixProduct661(const double * A, const double * B, double * C);

	void matrixProduct441(const double * A, const double * B, double * C);

	void matrixProduct331(const double * A, const double * B, double * C);

	void matrixProduct341(const double * A, const double * B, double * C);

	void matrixProduct231(const double * A, const double * B, double * C);

	void matrixProduct221(const double * A, const double * B, double * C);

	void matrixSum(double * A, double * B, double * C, const int m, const int n);

	void matrixSub(double * A, double * B, double * C, const int m, const int n);

	void matrixSum33(double * A, double * B, double * C);

	void matrixSub33(double * A, double * B, double * C);

	void matrixSum31(double * A, double * B, double * C);

	void matrixSub31(double * A, double * B, double * C);

	int projectPoint(const double * K, const double * X, double * Y);

	int transformAndProjectPoint(const double * K, const double * T, const double * X, double * Y);

	void transformPoint3d(const double * T, const double * P, double * Q);

	void inverseTransform(const double * T, double * T_inv);

	void matrixScale(double * A, double * AS, double a, int m, int n);

	double detSymm6x6(double * A);

	double norm(double * a, int m);

	void normalize(double * a, int m, int n);

	double dotProd3(double * a, double * b);
	
	double dotProd4(double * a, double * b);

	double dotProd(double * a, double * b, int m, int n);

	void skewMatFromVec(double * w, double * what);

	void skewMatToVec(double * what, double * w);
	
	void outerProd3(double * a, double * b, double * ab);

	void crossProd(const double * a, const double * b, double * c);

	void matrixRvecToRmat(const double * rvec, double * Rmat);

	int matrixRmatToRvec(const double * Rmat, double * rvec);

	void RotFromEuler(double * R, double ax, double ay, double az);
	
	void TfromRotTransl(const double * R, const double * t, double * T);

	void rotTranslFromT(const double * T, double * R, double * t);

	void TfromEuler(double * T, double ax, double ay, double az, double tx, double ty, double tz);

	void TfromTwist(const double * twist, double * T);
	
	int TwistFromT(const double * T, double * twist);

	void updatePoseTwist(double * T, double * dp);

	void TfromAngleAxis(const double * rotTrasl, double * T);

	int AngleAxisFromT(const double * T, double * rotTrasl);

	void updatePoseAngleAxis(double * T, double * dp);

	void adjustPoseVector_rodrigues_invZ(const double * posea, double * poseb, const bool from_tilde, const double s, const double znear, const double zfar, const double Xmax, const double Ymax, const double tz_ref, bool change_XY_with_Z, bool rotation_first);

	
	int fillConvexPoly_DepthTest(unsigned char * render_image,
									unsigned char render_image_color,
								  double * inv_Zbuffer,
								  int * polygon_id_map,
								  const unsigned char * stencil_mask,
								  const int xres,
								  const int yres,
								  const int nvert,
								  const double *vert,
								  const int *win,
								  const int poly_id = 0,
								  const double coeffx = 0,
								  const double coeffy = 0,
								  const double coeff0 = 0,
								  const double inv_znear = 0,
								  const double inv_zfar = 0);

	void poly_clip_to_view_frustum(int& nvert, double * poly, double * ptmp, double * frustum, int * edge_ids, int * edge_ids_tmp);

	void computeViewingFrustum(double * planes_frustum, const double * K, const double xmin, const double ymin, const double xmax, const double ymax, const double zmin, const double zmax);

	
	void printError(const int err_code);

	int loadImage(const char * title, unsigned char * img, int w, int h);

	void saveImage(const char * title, const unsigned char * img, const int w, const int h, const int c);

	int load_matrix(const char * fname, double * mat, int m, int n);

	void save_matrix(const char * fname, const double * mat, const int m, const int n);

	void print_matrix(const char * title, const double * mat, const int m, const int n);

	void print_matrix_float(const char * title, const float * mat, const int m, const int n);

	void print_matrix_int(const char * title, const int * mat, const int m, const int n);

	void print_matrix_short(const char * title, const short * mat, const int m, const int n);

	void print_matrix_signed_char(const char * title, const signed char * mat, const int m, const int n);

	void print_matrix_unsigned_char(const char * title, const unsigned char * mat, const int m, const int n);

}; // namespace DLRtracker

#endif /* COMMON_H_ */
