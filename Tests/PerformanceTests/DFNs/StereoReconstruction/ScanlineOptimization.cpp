/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ScanlineOptimization.cpp
 * @date 24/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Scanline Optimization.
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNs/PerformanceTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace PointCloudWrapper;


class ScanlineOptimizationTestInterface : public PerformanceTestInterface
	{
	public:
		ScanlineOptimizationTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);
		~ScanlineOptimizationTestInterface();

		void SetImageFilesPath(std::string leftImageFilePath, std::string rightImageFilePath);
		void SetReferenceDisparity(std::string referenceDisparityFilePath);
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		cv::Mat referenceDisparity;
		cv::Mat normalizedReferenceDisparity;
		bool saveDisparity;
		bool noReferenceDisparity;
		std::string leftImageFilePath;
		std::string rightImageFilePath;

		ScanlineOptimization* scanlineOptimization;
		void SetupMocksAndStubs();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();
	};

ScanlineOptimizationTestInterface::ScanlineOptimizationTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	scanlineOptimization = new ScanlineOptimization();
	SetDfn(scanlineOptimization);
	SetupMocksAndStubs();

	SetReferenceDisparity("../tests/Data/Images/chairDepth40.yml");
	saveDisparity = true;

	leftImageFilePath = "../tests/Data/Images/RectifiedChair40Left.png";
	rightImageFilePath = "../tests/Data/Images/RectifiedChair40Right.png";

	noReferenceDisparity = false;
	}

ScanlineOptimizationTestInterface::~ScanlineOptimizationTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(scanlineOptimization);
	}

void ScanlineOptimizationTestInterface::SetImageFilesPath(std::string leftImageFilePath, std::string rightImageFilePath)
	{
	this->leftImageFilePath = leftImageFilePath;
	this->rightImageFilePath = rightImageFilePath;
	}

void ScanlineOptimizationTestInterface::SetReferenceDisparity(std::string referenceDisparityFilePath)
	{
	if (referenceDisparityFilePath == "NULL" || referenceDisparityFilePath == "null" || referenceDisparityFilePath == "Null")
		{
		noReferenceDisparity = true;
		return;
		}

	cv::FileStorage file(referenceDisparityFilePath, cv::FileStorage::READ);
	file["depth"] >> referenceDisparity;

	cv::normalize(referenceDisparity, normalizedReferenceDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	std::stringstream stream;
	stream << "../tests/ConfigurationFiles/DFNs/StereoReconstruction/ReferenceDisparity.jpg"; 
	cv::imwrite(stream.str(), normalizedReferenceDisparity);
	}

void ScanlineOptimizationTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

bool ScanlineOptimizationTestInterface::SetNextInputs()
	{
	static unsigned time = 0;

	if (time == 0)
		{
		cv::Mat cvLeftImage = cv::imread(leftImageFilePath, cv::IMREAD_COLOR);
		cv::Mat cvRightImage = cv::imread(rightImageFilePath, cv::IMREAD_COLOR);
		ASSERT( cvLeftImage.cols > 0 && cvLeftImage.rows > 0, "Performance Test Error: bad input images");
		ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");
		ASSERT(noReferenceDisparity || referenceDisparity.size() == cvRightImage.size(), "Performance Test Error: reference disparity does not have same size as input images");

		MatToFrameConverter converter;
		FrameConstPtr leftFrame = converter.Convert(cvLeftImage);
		FrameConstPtr rightFrame = converter.Convert(cvRightImage);

		scanlineOptimization->leftImageInput(leftFrame);
		scanlineOptimization->rightImageInput(rightFrame);

		time++;
		return true;
		}
	
	return false;
	}

ScanlineOptimizationTestInterface::MeasuresMap ScanlineOptimizationTestInterface::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;

	MeasuresMap measuresMap;
	cv::Mat disparity = scanlineOptimization->disparityMatrixOutput(); //Disparity type is CV_16SC1
	cv::Mat normalizedDisparity;
	cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	/** The output disparity has always a black border on the left side of the image, here we compute the size of the black border as well as its ending column **/
	bool stop = false;
	unsigned firstValidColumn, numberOfValidColumns;
	for(unsigned column = 0; column < normalizedDisparity.cols && !stop; column++)
		{
		for(unsigned row = 0; row < normalizedDisparity.rows; row++)
			{
			stop = normalizedDisparity.at<uint8_t>(row, column) > 0;
			}
		if (stop)
			{
			firstValidColumn = column;
			}
		}
	numberOfValidColumns = normalizedDisparity.cols - firstValidColumn;

	/** Sometimes completely white images are in the output, this loop determines whether the image is completely white**/
	bool badOutput = true;
	for(unsigned column = firstValidColumn; column < normalizedDisparity.cols && badOutput; column++)
		{
		for(unsigned row = 0; row < normalizedDisparity.rows && badOutput; row++)
			{
			badOutput = normalizedDisparity.at<uint8_t>(row, column) == 255;
			}
		}
	measuresMap["OutputQuality"] = badOutput ? 0 : 1;	

	if (!noReferenceDisparity)
		{

		//This loop computes the difference between the computed disparity and the reference disparity after normalization of both disparities in the interval [0, 255]
		uint64_t cost = 0;
		unsigned pixelsNumber = 0;
		for(unsigned row = 0; row < normalizedReferenceDisparity.rows; row++)
			{
			for(unsigned column = 0; column < normalizedReferenceDisparity.cols; column++)
				{
				uint8_t referenceDisparityValue = normalizedReferenceDisparity.at<uint8_t>(row, column);
				if (referenceDisparityValue > 0)
					{
					unsigned validColumn = firstValidColumn + (column * numberOfValidColumns / normalizedReferenceDisparity.cols );
					uint8_t disparityValue = normalizedDisparity.at<uint8_t>(row, validColumn);
					cost += std::abs(static_cast<int16_t>(disparityValue) - static_cast<int16_t>(255 - referenceDisparityValue));	
					pixelsNumber++;
					}
				}
			}
		measuresMap["DisparityCost"] = static_cast<double>(cost) / static_cast<double>(pixelsNumber);

		}


	if (saveDisparity)
		{
		std::stringstream stream;
		stream << "../tests/ConfigurationFiles/DFNs/StereoReconstruction/Test_" << testId << ".jpg"; 
		cv::imwrite(stream.str(), normalizedDisparity);
		}

	PointCloudConstPtr outputCloud = scanlineOptimization->pointCloudOutput();
	delete(outputCloud);

	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::string configurationFileName = "ScanlineOptimization_Performance1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	ScanlineOptimizationTestInterface interface("../tests/ConfigurationFiles/DFNs/StereoReconstruction", configurationFileName, "ScanlineOptimizarionPerformance.txt");
	
	if (argc >= 5)
		{
		interface.SetImageFilesPath(argv[2], argv[3]);
		interface.SetReferenceDisparity(argv[4]);
		}

	interface.Run();
	};

/** @} */
