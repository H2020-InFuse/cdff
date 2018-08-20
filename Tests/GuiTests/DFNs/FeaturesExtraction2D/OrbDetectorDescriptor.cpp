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
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>


using namespace CDFF::DFN::FeaturesExtraction2D;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;


class OrbDetectorDescriptorTestInterface : public DFNTestInterface
	{
	public:
		OrbDetectorDescriptorTestInterface(std::string dfnName, int buttonWidth, int buttonHeight, std::string imageFilePath = DEFAULT_IMAGE_FILE_PATH);
		~OrbDetectorDescriptorTestInterface();
	protected:

	private:
		static const std::string DEFAULT_IMAGE_FILE_PATH;
		static const int ORB_DESCRIPTOR_SIZE;

		OrbDetectorDescriptor* orb;

		bool saveFeaturesToFile;
		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();

		void GetComponentRange(const VisualPointFeatureVector2D& featuresVector, int componentIndex, float& min, float& max);
	};

const std::string OrbDetectorDescriptorTestInterface::DEFAULT_IMAGE_FILE_PATH = "../../tests/Data/Images/DevonIslandLeft.ppm";
const int OrbDetectorDescriptorTestInterface::ORB_DESCRIPTOR_SIZE = 32;

OrbDetectorDescriptorTestInterface::OrbDetectorDescriptorTestInterface(std::string dfnName, int buttonWidth, int buttonHeight, std::string imageFilePath)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	orb = new OrbDetectorDescriptor();
	SetDFN(orb);
	saveFeaturesToFile = true;

	MatToFrameConverter converter;
	if (imageFilePath == DEFAULT_IMAGE_FILE_PATH)
		{
		cv::Mat doubleImage = cv::imread(imageFilePath, cv::IMREAD_COLOR);
		cvImage = doubleImage( cv::Rect(0, 0, doubleImage.cols/2, doubleImage.rows) );
		}
	else
		{
		cvImage = cv::imread(imageFilePath, cv::IMREAD_COLOR);
		}
	inputImage = converter.Convert(cvImage);
	orb->frameInput(*inputImage);
	outputWindowName = "Orb Detector Descriptor Result";
	}

OrbDetectorDescriptorTestInterface::~OrbDetectorDescriptorTestInterface()
	{
	delete(orb);
	delete(inputImage);
	}

void OrbDetectorDescriptorTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "EdgeThreshold", 31, 100);
	AddParameter("GeneralParameters", "FastThreshold", 20, 100);
	AddParameter("GeneralParameters", "FirstLevel", 0, 2);
	AddParameter("GeneralParameters", "MaxFeaturesNumber", 1024, 1500, 10);
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

	const VisualPointFeatureVector2D& featuresVector = orb->featuresOutput();
	PRINT_TO_LOG("Number of features", GetNumberOfPoints(featuresVector) );
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	float minR, maxR, minG, maxG, minB, maxB;
	GetComponentRange(featuresVector, featureIndexForRedColor, minR, maxR);
	GetComponentRange(featuresVector, featureIndexForGreenColor, minG, maxG);
	GetComponentRange(featuresVector, featureIndexForBlueColor, minB, maxB);

	for(int featureIndex = 0; featureIndex < GetNumberOfPoints(featuresVector); featureIndex++)
		{
		ASSERT(GetNumberOfDescriptorComponents(featuresVector, featureIndex) == ORB_DESCRIPTOR_SIZE, "Orb descriptor size does not match size of received feature");
		for(int componentIndex = 0; componentIndex < ORB_DESCRIPTOR_SIZE; componentIndex++)
			{
			ASSERT(GetDescriptorComponent(featuresVector, featureIndex, componentIndex) == GetDescriptorComponent(featuresVector, featureIndex, componentIndex), "Invalid Descriptor");
			}

		cv::Point drawPoint(GetXCoordinate(featuresVector, featureIndex), GetYCoordinate(featuresVector, featureIndex) );

		int r = 255*( (GetDescriptorComponent(featuresVector, featureIndex, featureIndexForRedColor) - minR)/(maxR - minR) );
		int g = 255*( (GetDescriptorComponent(featuresVector, featureIndex, featureIndexForGreenColor) - minG)/(maxG - minG) );
		int b = 255*( (GetDescriptorComponent(featuresVector, featureIndex, featureIndexForBlueColor) - minB)/(maxB - minB) );

		cv::circle(outputImage, drawPoint, 5, cv::Scalar(r, g, b), 2, 8, 0);
		}

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of features detected: ", GetNumberOfPoints(featuresVector) );

	if (saveFeaturesToFile)
		{
		cv::Mat keypointsMatrix( GetNumberOfPoints(featuresVector), 2, CV_16UC1);
		cv::Mat descriptorsMatrix( GetNumberOfPoints(featuresVector), ORB_DESCRIPTOR_SIZE, CV_32FC1);
		for(int featureIndex = 0; featureIndex < GetNumberOfPoints(featuresVector); featureIndex++)
			{
			keypointsMatrix.at<uint16_t>(featureIndex, 0) = GetXCoordinate(featuresVector, featureIndex);
			keypointsMatrix.at<uint16_t>(featureIndex, 1) = GetYCoordinate(featuresVector, featureIndex);
			for(int componentIndex = 0; componentIndex < ORB_DESCRIPTOR_SIZE; componentIndex++)
				{
				descriptorsMatrix.at<float>(featureIndex, componentIndex) = GetDescriptorComponent(featuresVector, featureIndex, componentIndex);
				}
			}
		cv::FileStorage file("../../tests/Data/Images/OrbFeaturesScene1.yml", cv::FileStorage::WRITE);
		file << "keypoints" << keypointsMatrix;
		file << "descriptors" << descriptorsMatrix;
		}
	}

void OrbDetectorDescriptorTestInterface::GetComponentRange(const VisualPointFeatureVector2D& featuresVector, int componentIndex, float& min, float& max)
	{
	if ( GetNumberOfPoints(featuresVector) == 0 )
		{
		return;
		}

	min = GetDescriptorComponent(featuresVector, 0, componentIndex);
	max = min;
	for(int featureIndex = 1; featureIndex < GetNumberOfPoints(featuresVector); featureIndex++)
		{
		ASSERT(GetNumberOfDescriptorComponents(featuresVector, featureIndex) == ORB_DESCRIPTOR_SIZE, "Orb descriptor size does not match size of received feature");
		if (max < GetDescriptorComponent(featuresVector, featureIndex, componentIndex) )
			{
			max = GetDescriptorComponent(featuresVector, featureIndex, componentIndex);
			}
		if (min > GetDescriptorComponent(featuresVector, featureIndex, componentIndex) )
			{
			min = GetDescriptorComponent(featuresVector, featureIndex, componentIndex);
			}
		}
	if (max == min)
		{
		max = min + 1;
		}	
	}


int main(int argc, char** argv)
	{
	OrbDetectorDescriptorTestInterface* interface;
	if (argc > 1)
		{
		interface = new OrbDetectorDescriptorTestInterface("Orb", 100, 40, argv[1]);
		}
	else
		{
		interface = new OrbDetectorDescriptorTestInterface("Orb", 100, 40);
		}
	interface->Run();
	delete(interface);
	};

/** @} */
