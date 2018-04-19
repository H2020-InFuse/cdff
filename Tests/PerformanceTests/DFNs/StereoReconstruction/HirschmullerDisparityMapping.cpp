/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 19/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Hirschmuller Disparity Mapping.
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
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
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


class DisparityMappingTestInterface : public PerformanceTestInterface
	{
	public:
		DisparityMappingTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);
		~DisparityMappingTestInterface();

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
		std::string leftImageFilePath;
		std::string rightImageFilePath;

		HirschmullerDisparityMapping* disparityMapping;
		void SetupMocksAndStubs();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();
	};

DisparityMappingTestInterface::DisparityMappingTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	disparityMapping = new HirschmullerDisparityMapping();
	SetDfn(disparityMapping);
	SetupMocksAndStubs();

	SetReferenceDisparity("../tests/Data/Images/chairDepth40.yml");
	saveDisparity = true;

	leftImageFilePath = "../tests/Data/Images/RectifiedChair40Left.png";
	rightImageFilePath = "../tests/Data/Images/RectifiedChair40Right.png";
	}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(disparityMapping);
	}

void DisparityMappingTestInterface::SetImageFilesPath(std::string leftImageFilePath, std::string rightImageFilePath)
	{
	this->leftImageFilePath = leftImageFilePath;
	this->rightImageFilePath = rightImageFilePath;
	}

void DisparityMappingTestInterface::SetReferenceDisparity(std::string referenceDisparityFilePath)
	{
	cv::FileStorage file(referenceDisparityFilePath, cv::FileStorage::READ);
	file["depth"] >> referenceDisparity;

	cv::normalize(referenceDisparity, normalizedReferenceDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	std::stringstream stream;
	stream << "../tests/ConfigurationFiles/DFNs/StereoReconstruction/ReferenceDisparity.jpg"; 
	cv::imwrite(stream.str(), normalizedReferenceDisparity);
	}

void DisparityMappingTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

bool DisparityMappingTestInterface::SetNextInputs()
	{
	static unsigned time = 0;

	if (time == 0)
		{
		cv::Mat cvLeftImage = cv::imread(leftImageFilePath, cv::IMREAD_COLOR);
		cv::Mat cvRightImage = cv::imread(rightImageFilePath, cv::IMREAD_COLOR);
		ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");
		ASSERT( referenceDisparity.size() == cvRightImage.size(), "Performance Test Error: reference disparity does not have same size as input images");

		MatToFrameConverter converter;
		FrameConstPtr leftFrame = converter.Convert(cvLeftImage);
		FrameConstPtr rightFrame = converter.Convert(cvRightImage);

		disparityMapping->leftImageInput(leftFrame);
		disparityMapping->rightImageInput(rightFrame);

		time++;
		return true;
		}
	
	return false;
	}

DisparityMappingTestInterface::MeasuresMap DisparityMappingTestInterface::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;

	MeasuresMap measuresMap;
	cv::Mat disparity = disparityMapping->disparityMatrixOutput(); //Disparity type is CV_16SC1
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


	if (saveDisparity)
		{
		std::stringstream stream;
		stream << "../tests/ConfigurationFiles/DFNs/StereoReconstruction/Test_" << testId << ".jpg"; 
		cv::imwrite(stream.str(), normalizedDisparity);
		}

	PointCloudConstPtr outputCloud = disparityMapping->pointCloudOutput();
	delete(outputCloud);

	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::string configurationFileName = "HirschmullerDisparityMapping_Performance1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	DisparityMappingTestInterface interface("../tests/ConfigurationFiles/DFNs/StereoReconstruction", configurationFileName, "HirschmullerOutput.txt");
	
	if (argc >= 5)
		{
		interface.SetImageFilesPath(argv[2], argv[3]);
		interface.SetReferenceDisparity(argv[4]);
		}

	interface.Run();
	};

/** @} */
