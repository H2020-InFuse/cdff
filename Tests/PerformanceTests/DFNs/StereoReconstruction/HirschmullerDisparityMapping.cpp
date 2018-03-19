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
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		cv::Mat referenceDisparity;
		bool saveDisparity;

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

	referenceDisparity = cv::imread("../tests/Data/Images/ReferenceDisparity.png", cv::IMREAD_COLOR);
	saveDisparity = false;
	}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(disparityMapping);
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
		cv::Mat cvLeftImage = cv::imread("../tests/Data/Images/RectifiedLeft.png", cv::IMREAD_COLOR);
		cv::Mat cvRightImage = cv::imread("../tests/Data/Images/RectifiedRight.png", cv::IMREAD_COLOR);
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
	cv::Mat disparity = disparityMapping->disparityMatrixOutput();

	//The disparity matrix is shifted to the right with respect to the reference disparity image.
	const unsigned COLUMN_SHIFT = 40;
	uint64_t cost = 0;
	for(unsigned row = 0; row < referenceDisparity.rows; row++)
		{
		for(unsigned column = COLUMN_SHIFT; column < referenceDisparity.cols; column++)
			{
			cv::Vec3b color = referenceDisparity.at<cv::Vec3b>(row, column-COLUMN_SHIFT);
			int16_t disparityValue = disparity.at<int16_t>(row, column);
			if (color[0] == color[1] && color[1] == color[2])
				{
				cost += std::abs(static_cast<int32_t>(disparityValue) - static_cast<int32_t>(color[0]));	
				}
			}
		}
	unsigned pixelsNumber = referenceDisparity.cols * referenceDisparity.rows;
	measuresMap["DisparityCost"] = static_cast<double>(cost) / static_cast<double>(pixelsNumber);


	if (saveDisparity)
		{
		cv::Mat normalizedDisparity;
		cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);

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
	DisparityMappingTestInterface interface("../tests/ConfigurationFiles/DFNs/StereoReconstruction", "HirschmullerDisparityMapping_Performance1.yaml", "HirschmullerOutput.txt");
	interface.Run();
	};

/** @} */
