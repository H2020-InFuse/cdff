/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageFiltering.cpp
 * @date 25/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the class ImageFilteringTestInterface.
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
#include "ImageFiltering.hpp"
#include <fstream>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageFilteringTestInterface::ImageFilteringTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, ImageFilteringInterface* filter)
	: PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	ASSERT(filter != NULL, "The filter was not correctly defined");
	this->filter = filter;
	SetDfn(this->filter);

	saveOutput = false;

	baseImageFolder = "../tests/Data/Images";
	imagesListFileName = "FilterImagesList.txt";
	imagesOutputFileBaseName = "";
	imagesOutputExtension = "";

	std::string uniqueStartFile = "RectifiedChair40Left.png";
	imageFileNamesList.push_back(uniqueStartFile);
	}

ImageFilteringTestInterface::~ImageFilteringTestInterface()
	{
	}

void ImageFilteringTestInterface::SetImageFilePath(std::string baseImageFolder, std::string imagesListFileName)
	{
	this->baseImageFolder = baseImageFolder;
	this->imagesListFileName = imagesListFileName;
	ReadImageFileNamesList();
	}

void ImageFilteringTestInterface::SetOutputFile(std::string imagesOutputFileBaseName, std::string imagesOutputExtension)
	{
	saveOutput = true;
	this->imagesOutputFileBaseName = imagesOutputFileBaseName;
	this->imagesOutputExtension = imagesOutputExtension;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ImageFilteringTestInterface::ReadImageFileNamesList()
	{
	imageFileNamesList.clear();

	std::stringstream listFilePath;
	listFilePath << baseImageFolder << "/" << imagesListFileName;

	std::ifstream file(listFilePath.str().c_str());
	
	while(file.good())
		{
		std::string newString;
		file >> newString;
		if (file.good())
			{
			imageFileNamesList.push_back(newString);
			}
		}

	file.close();
	}

bool ImageFilteringTestInterface::SetNextInputs()
	{
	static unsigned time = 0;

	if (time < imageFileNamesList.size())
		{
		std::stringstream imageFilePath;
		imageFilePath << baseImageFolder << "/" << imageFileNamesList.at(time);

		cv::Mat cvImage = cv::imread(imageFilePath.str(), cv::IMREAD_COLOR);
		ASSERT( cvImage.cols > 0 && cvImage.rows > 0, "Performance Test Error: bad input images");

		if (inputFrame != NULL)
			{
			delete(inputFrame);
			}
		MatToFrameConverter converter;
		inputFrame = converter.Convert(cvImage);

		filter->imageInput(*inputFrame);

		time++;
		return true;
		}
	
	return false;
	}

ImageFilteringTestInterface::MeasuresMap ImageFilteringTestInterface::ExtractMeasures()
	{
	//testId has to start from 1 to match the inputId as saved in the output file.
	static unsigned testId = 0;
	testId++;

	FramePtr outputFrame = NewFrame();
	Copy( filter->imageOutput(), *outputFrame);

	FrameToMatConverter converter;
	cv::Mat cvImage = converter.Convert(outputFrame); 

	if (saveOutput)
		{
		unsigned fileNumber = testId - 1; // This is to match DLR dataset
		std::stringstream fileStream;
		fileStream << baseImageFolder << "/" << imagesOutputFileBaseName << (fileNumber < 100 ? "0" : "") << (fileNumber < 10 ? "0" : "") << fileNumber << imagesOutputExtension; 
		std::string fileString = fileStream.str();
		PRINT_TO_LOG("Saving", fileString);
		cv::imwrite(fileString, cvImage);
		}
	delete(outputFrame);


	MeasuresMap measuresMap;
	return measuresMap;
	}

/** @} */
