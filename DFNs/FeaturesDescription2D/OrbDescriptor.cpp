/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "OrbDescriptor.hpp"
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;

namespace CDFF
{
namespace DFN
{
namespace FeaturesDescription2D
{

OrbDescriptor::OrbDescriptor()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<int>("GeneralParameters", "EdgeThreshold", parameters.edgeThreshold, DEFAULT_PARAMETERS.edgeThreshold);
	parametersHelper.AddParameter<int>("GeneralParameters", "FastThreshold", parameters.fastThreshold, DEFAULT_PARAMETERS.fastThreshold);
	parametersHelper.AddParameter<int>("GeneralParameters", "FirstLevel", parameters.firstLevel, DEFAULT_PARAMETERS.firstLevel);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaxFeaturesNumber", parameters.maxFeaturesNumber, DEFAULT_PARAMETERS.maxFeaturesNumber);

	parametersHelper.AddParameter<int>("GeneralParameters", "LevelsNumber", parameters.levelsNumber, DEFAULT_PARAMETERS.levelsNumber);
	parametersHelper.AddParameter<int>("GeneralParameters", "PatchSize", parameters.patchSize, DEFAULT_PARAMETERS.patchSize);
	parametersHelper.AddParameter<double>("GeneralParameters", "ScaleFactor", parameters.scaleFactor, DEFAULT_PARAMETERS.scaleFactor);
	parametersHelper.AddParameter<int>("GeneralParameters", "ScoreType", parameters.scoreType, DEFAULT_PARAMETERS.scoreType);

	parametersHelper.AddParameter<int>("GeneralParameters", "SizeOfBrightnessTestSet", parameters.sizeOfBrightnessTestSet, DEFAULT_PARAMETERS.sizeOfBrightnessTestSet);

	configurationFilePath = "";
}

OrbDescriptor::~OrbDescriptor()
{
}

void OrbDescriptor::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void OrbDescriptor::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inFrame);
	std::vector<cv::KeyPoint> keypointsVector = Convert(inFeatures);

	// Process data
	ValidateInputs(inputImage, keypointsVector);
	cv::Mat orbFeatures = ComputeOrbFeatures(inputImage, keypointsVector);

	// Write data to output port
	VisualPointFeatureVector2DConstPtr tmp = matToVisualPointFeatureVector2D.Convert(orbFeatures);
	Copy(*tmp, outFeatures);
	delete(tmp);
}

const OrbDescriptor::OrbOptionsSet OrbDescriptor::DEFAULT_PARAMETERS =
{
	/*.edgeThreshold =*/ 31,
	/*.fastThreshold =*/ 20,
	/*.firstLevel =*/ 0,
	/*.maxFeaturesNumber =*/ 500,
	/*.levelsNumber =*/ 8,
	/*.patchSize =*/ 31,
	/*.scaleFactor =*/ 1.2,
	/*.scoreType =*/ 0,
	/*.sizeOfBrightnessTestSet =*/ 2
};

int OrbDescriptor::ConvertToScoreType(const std::string& scoreType)
{
	if (scoreType == "HarrisScore" || scoreType == "0")
		{
		return cv::ORB::HARRIS_SCORE;
		}
	if (scoreType == "FastScore" || scoreType == "1")
		{
		return cv::ORB::FAST_SCORE;
		}
	ASSERT(false, "Orb Detector Descriptor Configuration Error: Score type should be either HarrisScore or FastScore (1, or 2)");
}

std::vector<cv::KeyPoint> OrbDescriptor::Convert(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& featuresVector)
{
	std::vector<cv::KeyPoint> keypointsVector;
	for (int featureIndex = 0; featureIndex < GetNumberOfPoints(featuresVector); featureIndex++)
	{
		int descriptorSize = GetNumberOfDescriptorComponents(featuresVector, featureIndex);

		float size = (descriptorSize > 0) ? GetDescriptorComponent(featuresVector, featureIndex, 0) : 1;
		float angle = (descriptorSize > 1) ? GetDescriptorComponent(featuresVector, featureIndex, 1) : -1;
		float response = (descriptorSize > 2) ? GetDescriptorComponent(featuresVector, featureIndex, 2) : 0;
		float octave = (descriptorSize > 3) ? GetDescriptorComponent(featuresVector, featureIndex, 3) : 0;
		float id = (descriptorSize > 4) ? GetDescriptorComponent(featuresVector, featureIndex, 4) : -1;

		cv::KeyPoint newPoint(
			GetXCoordinate(featuresVector, featureIndex),
			GetYCoordinate(featuresVector, featureIndex),
			size,
			angle,
			response,
			octave,
			id
		);
		keypointsVector.push_back(newPoint);
	}

	return keypointsVector;
}

cv::Mat OrbDescriptor::ComputeOrbFeatures(cv::Mat inputImage, std::vector<cv::KeyPoint> keypointsVector)
{
	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	orb->setEdgeThreshold(parameters.edgeThreshold);
	orb->setFastThreshold(parameters.fastThreshold);
	orb->setFirstLevel(parameters.firstLevel);
	orb->setMaxFeatures(parameters.maxFeaturesNumber);
	orb->setNLevels(parameters.levelsNumber);
	orb->setPatchSize(parameters.patchSize);
	orb->setScaleFactor(parameters.scaleFactor);
	orb->setScoreType(parameters.scoreType);
	orb->setWTA_K(parameters.sizeOfBrightnessTestSet);

	cv::Mat descriptorsMatrix;
	cv::Mat mask = cv::Mat();
	orb->detectAndCompute(inputImage, mask, keypointsVector, descriptorsMatrix, true);
	ASSERT(keypointsVector.size() == descriptorsMatrix.rows, "Orb Error: keypoints vector size does not match descriptorMatrix rows number");

	if (keypointsVector.size() == 0)
	{
		return cv::Mat();
	}

	cv::Mat orbFeaturesMatrix(keypointsVector.size(), descriptorsMatrix.cols + 2, CV_32FC1);
	cv::Mat descriptorsSubmatrix = orbFeaturesMatrix( cv::Rect(2, 0, orbFeaturesMatrix.cols-2, orbFeaturesMatrix.rows) );
	descriptorsMatrix.convertTo(descriptorsSubmatrix, CV_32FC1);

	for (unsigned pointIndex = 0; pointIndex < keypointsVector.size(); pointIndex++)
	{
		orbFeaturesMatrix.at<float>(pointIndex, 0) = keypointsVector.at(pointIndex).pt.x;
		orbFeaturesMatrix.at<float>(pointIndex, 1) = keypointsVector.at(pointIndex).pt.y;
	}

	return orbFeaturesMatrix;
}

void OrbDescriptor::ValidateParameters()
{
	ASSERT(parameters.edgeThreshold > 0, "Orb Detector Descriptor Configuration Error: edge threshold should be strictly positive");
	ASSERT(parameters.fastThreshold > 0, "Orb Detector Descriptor Configuration Error: fast threshold should be strictly positive");
	ASSERT(parameters.firstLevel == 0, "Orb Detector Descriptor Configuration Error: first level should be 0");
	ASSERT(parameters.maxFeaturesNumber > 0, "Orb Detector Descriptor Configuration Error: max features number should be strictly positive");
	ASSERT(parameters.levelsNumber > 0, "Orb Detector Descriptor Configuration Error: pyramid levels number should be strictly positive");
	ASSERT(parameters.patchSize > 0, "Orb Detector Descriptor Configuration Error: patch size should be strictly positive");
	ASSERT(parameters.scaleFactor > 1, "Orb Detector Descriptor Configuration Error: scale factor should be greater than 1");
	ASSERT(parameters.scoreType == cv::ORB::HARRIS_SCORE || parameters.scoreType == cv::ORB::FAST_SCORE, "Orb Detector Descriptor Configuration Error: scoreType should be HarrisScore or FastScore");
	ASSERT(parameters.sizeOfBrightnessTestSet >= 2 && parameters.sizeOfBrightnessTestSet <= 4, "Orb Detector Descriptor Configuration Error: size of brightness test set should be 2, 3, or 4");
}

void OrbDescriptor::ValidateInputs(cv::Mat inputImage, const std::vector<cv::KeyPoint>& keypointsVector)
{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "OrbDetectorDescriptor error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "OrbDetectorDescriptor error: input image is empty");
}

}
}
}

/** @} */
