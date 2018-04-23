/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionRectification.cpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Image Undistortion Rectification.
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
#include <ImageFiltering/ImageUndistortionRectification.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <FrameToMatConverter.hpp>
#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNs/PerformanceTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;


class ImageUndistortionRectificationTestInterface : public PerformanceTestInterface
	{
	public:
		ImageUndistortionRectificationTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);
		~ImageUndistortionRectificationTestInterface();

		void SetImageFilePath(std::string imageFilePath);
		void SetOutputFile(std::string outputFileBasePath, std::string outputFileExtension);
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;

		bool saveOutput;
		std::string imageFilePath;
		std::string outputFileBasePath;
		std::string outputFileExtension;

		ImageUndistortionRectification* undistortion;
		void SetupMocksAndStubs();

		bool SetNextInputs();
		MeasuresMap ExtractMeasures();
	};

ImageUndistortionRectificationTestInterface::ImageUndistortionRectificationTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	undistortion = new ImageUndistortionRectification();
	SetDfn(undistortion);
	SetupMocksAndStubs();

	saveOutput = false;

	imageFilePath = "../tests/Data/Images/RectifiedChair40Left.png";
	outputFileBasePath = "../tests/ConfigurationFiles/DFNs/StereoReconstruction/Test_";
	outputFileExtension = ".jpg";
	}

ImageUndistortionRectificationTestInterface::~ImageUndistortionRectificationTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(undistortion);
	}

void ImageUndistortionRectificationTestInterface::SetImageFilePath(std::string imageFilePath)
	{
	this->imageFilePath = imageFilePath;
	}

void ImageUndistortionRectificationTestInterface::SetOutputFile(std::string outputFileBasePath, std::string outputFileExtension)
	{
	saveOutput = true;
	this->outputFileBasePath = outputFileBasePath;
	this->outputFileExtension = outputFileExtension;
	}

void ImageUndistortionRectificationTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

bool ImageUndistortionRectificationTestInterface::SetNextInputs()
	{
	static unsigned time = 0;

	if (time == 0)
		{
		cv::Mat cvImage = cv::imread(imageFilePath, cv::IMREAD_COLOR);
		ASSERT( cvImage.cols > 0 && cvImage.rows > 0, "Performance Test Error: bad input images");

		MatToFrameConverter converter;
		FrameConstPtr frame = converter.Convert(cvImage);

		undistortion->imageInput(frame);

		time++;
		return true;
		}
	
	return false;
	}

ImageUndistortionRectificationTestInterface::MeasuresMap ImageUndistortionRectificationTestInterface::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;

	FrameConstPtr outputFrame = undistortion->filteredImageOutput();
	FrameToMatConverter converter;
	cv::Mat cvImage = converter.Convert(outputFrame); 

	if (saveOutput)
		{
		std::stringstream stream;
		stream << outputFileBasePath << testId << outputFileExtension; 
		cv::imwrite(stream.str(), cvImage);
		}
	delete(outputFrame);

	MeasuresMap measuresMap;
	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::string configurationFileName = "ImageUndistortionRectification_DlrSunLeft.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	ImageUndistortionRectificationTestInterface interface("../tests/ConfigurationFiles/DFNs/ImageFiltering", configurationFileName, "ImageUndistortionRectificationOutput.txt");
	
	if (argc >= 3)
		{
		interface.SetImageFilePath(argv[2]);
		}

	if (argc >= 5)
		{
		interface.SetOutputFile(argv[3], argv[4]);
		}

	interface.Run();
	};

/** @} */
