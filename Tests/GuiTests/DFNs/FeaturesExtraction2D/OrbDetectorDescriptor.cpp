/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptor.cpp
 * @date 23/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN OrbDetectorDescriptor.
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
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;


class OrbDetectorDescriptorTestInterface : public DFNTestInterface
	{
	public:
		OrbDetectorDescriptorTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~OrbDetectorDescriptorTestInterface();
	protected:

	private:
		static const int ORB_DESCRIPTOR_SIZE;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockOutputConverter;
		OrbDetectorDescriptor* orb;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();

		void GetComponentRange(VisualPointFeatureVector2DConstPtr featuresVector, int componentIndex, float& min, float& max);
	};

const int OrbDetectorDescriptorTestInterface::ORB_DESCRIPTOR_SIZE = 32;

OrbDetectorDescriptorTestInterface::OrbDetectorDescriptorTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	orb = new OrbDetectorDescriptor();
	SetDFN(orb);

	MatToFrameConverter converter;
	cvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	inputImage = converter.Convert(cvImage);
	orb->imageInput(inputImage);
	outputWindowName = "Orb Detector Descriptor Result";
	}

OrbDetectorDescriptorTestInterface::~OrbDetectorDescriptorTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(orb);
	delete(inputImage);
	}

void OrbDetectorDescriptorTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void OrbDetectorDescriptorTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "EdgeThreshold", 31, 100);
	AddParameter("GeneralParameters", "FastThreshold", 20, 100);
	AddParameter("GeneralParameters", "FirstLevel", 0, 2);
	AddParameter("GeneralParameters", "MaxFeaturesNumber", 500, 1000, 10);
	AddParameter("GeneralParameters", "LevelsNumber", 8, 20);
	AddParameter("GeneralParameters", "PatchSize", 31, 100);
	AddParameter("GeneralParameters", "ScaleFactor", 1.2, 10, 0.1);
	AddParameter("GeneralParameters", "ScoreType", 0, 2);
	AddParameter("GeneralParameters", "SizeOfBrightnessTestSet", 2, 4);
	}

void OrbDetectorDescriptorTestInterface::DisplayResult()
	{
	const int featureIndexForRedColor = 2;
	const int featureIndexForGreenColor = 4;
	const int featureIndexForBlueColor = 5;

	VisualPointFeatureVector2DConstPtr featuresVector= orb->featuresSetOutput();
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	float minR, maxR, minG, maxG, minB, maxB;
	GetComponentRange(featuresVector, featureIndexForRedColor, minR, maxR);
	GetComponentRange(featuresVector, featureIndexForGreenColor, minG, maxG);
	GetComponentRange(featuresVector, featureIndexForBlueColor, minB, maxB);

	for(int featureIndex = 0; featureIndex < GetNumberOfPoints(*featuresVector); featureIndex++)
		{
		ASSERT(GetNumberOfDescriptorComponents(*featuresVector, featureIndex) == ORB_DESCRIPTOR_SIZE, "Orb descriptor size does not match size of received feature");
		for(int componentIndex = 0; componentIndex < ORB_DESCRIPTOR_SIZE; componentIndex++)
			{
			ASSERT(GetDescriptorComponent(*featuresVector, featureIndex, componentIndex) == GetDescriptorComponent(*featuresVector, featureIndex, componentIndex), "Invalid Descriptor");
			}

		cv::Point drawPoint(GetXCoordinate(*featuresVector, featureIndex), GetYCoordinate(*featuresVector, featureIndex) );

		int r = 255*( (GetDescriptorComponent(*featuresVector, featureIndex, featureIndexForRedColor) - minR)/(maxR - minR) );
		int g = 255*( (GetDescriptorComponent(*featuresVector, featureIndex, featureIndexForGreenColor) - minG)/(maxG - minG) );
		int b = 255*( (GetDescriptorComponent(*featuresVector, featureIndex, featureIndexForBlueColor) - minB)/(maxB - minB) );

		cv::circle(outputImage, drawPoint, 5, cv::Scalar(r, g, b), 2, 8, 0);
		}

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of features detected: ", GetNumberOfPoints(*featuresVector) );

	delete(featuresVector);
	}

void OrbDetectorDescriptorTestInterface::GetComponentRange(VisualPointFeatureVector2DConstPtr featuresVector, int componentIndex, float& min, float& max)
	{
	if ( GetNumberOfPoints(*featuresVector) == 0 )
		{
		return;
		}

	min = GetDescriptorComponent(*featuresVector, 0, componentIndex);
	max = min;
	for(int featureIndex = 1; featureIndex < GetNumberOfPoints(*featuresVector); featureIndex++)
		{
		ASSERT(GetNumberOfDescriptorComponents(*featuresVector, featureIndex) == ORB_DESCRIPTOR_SIZE, "Orb descriptor size does not match size of received feature");
		if (max < GetDescriptorComponent(*featuresVector, featureIndex, componentIndex) )
			{
			max = GetDescriptorComponent(*featuresVector, featureIndex, componentIndex);
			}
		if (min > GetDescriptorComponent(*featuresVector, featureIndex, componentIndex) )
			{
			min = GetDescriptorComponent(*featuresVector, featureIndex, componentIndex);
			}
		}
	if (max == min)
		{
		max = min + 1;
		}	
	}


int main(int argc, char** argv)
	{
	OrbDetectorDescriptorTestInterface interface("Orb", 100, 40);
	interface.Run();
	};

/** @} */
