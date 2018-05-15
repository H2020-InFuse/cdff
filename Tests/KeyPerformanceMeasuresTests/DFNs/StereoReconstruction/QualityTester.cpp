/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file QualityTester.cpp
 * @date 14/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the QualityTester class.
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
#include "QualityTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>

using namespace dfn_ci;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace FrameWrapper;
using namespace Common;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
QualityTester::QualityTester() 
	{
	inputLeftFrame = NULL;
	inputRightFrame = NULL;
	outputPointCloud = NULL;

	inputImagesWereLoaded = false;
	outputPointCloudWasLoaded = false;
	outliersReferenceWasLoaded = false;
	dfnExecuted = false;
	dfnWasLoaded = false;

	SetUpMocksAndStubs();
	}

QualityTester::~QualityTester()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubCloudCache);
	delete(mockCloudConverter);
	
	DELETE_IF_NOT_NULL(inputLeftFrame);
	DELETE_IF_NOT_NULL(inputRightFrame);
	DELETE_IF_NOT_NULL(outputPointCloud);
	}

void QualityTester::SetDfn(std::string configurationFilePath, dfn_ci::StereoReconstructionInterface* dfn)
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;

	ConfigureDfn();
	dfnWasLoaded = true;
	}

void QualityTester::SetInputFilesPaths(std::string inputLeftImageFilePath, std::string inputRightImageFilePath)
	{
	this->inputLeftImageFilePath = inputLeftImageFilePath;
	this->inputRightImageFilePath = inputRightImageFilePath;

	LoadInputImage(inputLeftImageFilePath, inputLeftFrame);
	LoadInputImage(inputRightImageFilePath, inputRightFrame);
	inputImagesWereLoaded = true;
	}

void QualityTester::SetOutputFilePath(std::string outputPointCloudFilePath)
	{
	this->outputPointCloudFilePath = outputPointCloudFilePath;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(outputPointCloudFilePath, *pclPointCloud);
	
	DELETE_IF_NOT_NULL(outputPointCloud);
	outputPointCloud = inverseCloudConverter.Convert(pclPointCloud);

	outputPointCloudWasLoaded = true;
	}

void QualityTester::SetOutliersFilePath(std::string outliersReferenceFilePath)
	{
	this->outliersReferenceFilePath = outliersReferenceFilePath;

	LoadOutliersReference();
	outliersReferenceWasLoaded = true;
	}

void QualityTester::ExecuteDfn()
	{
	ASSERT(inputImagesWereLoaded && dfnWasLoaded, "Cannot execute DFN if input images or the dfn itself are not loaded");
	dfn->leftImageInput(inputLeftFrame);
	dfn->rightImageInput(inputRightFrame);
	dfn->process();

	DELETE_IF_NOT_NULL(outputPointCloud);
	outputPointCloud = dfn->pointCloudOutput();

	PRINT_TO_LOG("Point cloud size is", GetNumberOfPoints(*outputPointCloud));
	dfnExecuted = true;
	}

bool QualityTester::IsQualitySufficient(float outliersPercentageThreshold)
	{
	ASSERT(outputPointCloudWasLoaded && outliersReferenceWasLoaded, "Error: you have to load cloud and outliers");

	PRINT_TO_LOG("Point cloud size is", GetNumberOfPoints(*outputPointCloud));
	PRINT_TO_LOG("Number of outliers is", outliersMatrix.rows);
	
	float outlierPercentage = ((float)outliersMatrix.rows) / ( (float)GetNumberOfPoints(*outputPointCloud) );
	PRINT_TO_LOG("Outlier percentage is: ", outlierPercentage);

	bool withinOutliersThreshold = outlierPercentage < outliersPercentageThreshold;

	if (!withinOutliersThreshold)
		{
		PRINT_TO_LOG("The outliers percentage is above the threshold", outliersPercentageThreshold);
		}

	return (withinOutliersThreshold);
	}

void QualityTester::SaveOutputPointCloud()
	{
	ASSERT(dfnExecuted, "Cannot save output cloud if DFN was not executed before");
	
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = pointCloudConverter.Convert(outputPointCloud);

	pcl::PLYWriter writer;
	writer.write(outputPointCloudFilePath, *pclPointCloud, true);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void QualityTester::SetUpMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>;
	mockFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>;
	mockInverseFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr>();
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

void QualityTester::LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame)
	{
	cv::Mat cvImage = cv::imread(filePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(frame);	
	frame = frameConverter.Convert(cvImage);
	}

void QualityTester::LoadOutliersReference()
	{
	cv::FileStorage opencvFile(outliersReferenceFilePath, cv::FileStorage::READ);
	opencvFile["OutliersMatrix"] >> outliersMatrix;
	opencvFile.release();
	ASSERT(outliersMatrix.rows > 0 && outliersMatrix.cols == 1 && outliersMatrix.type() == CV_32SC1, "Error: reference outliers are invalid");
	}

void QualityTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

/** @} */
