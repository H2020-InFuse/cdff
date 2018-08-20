/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FlannMatcher.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN Flann Matcher.
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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <BaseTypes.hpp>

using namespace CDFF::DFN::FeaturesMatching2D;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;

class FlannMatcherTestInterface : public DFNTestInterface
	{
	public:
		FlannMatcherTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~FlannMatcherTestInterface();

	private:
		FlannMatcher* flann;

		cv::Mat cvImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
	};

FlannMatcherTestInterface::FlannMatcherTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	flann = new FlannMatcher();
	SetDFN(flann);

	cvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	orb->setEdgeThreshold(31);
	orb->setFastThreshold(20);
	orb->setFirstLevel(0);
	orb->setMaxFeatures(500);
	orb->setNLevels(8);
	orb->setPatchSize(31);
	orb->setScaleFactor(1.2);
	orb->setScoreType(0);
	orb->setWTA_K(2);

	std::vector<cv::KeyPoint> keypointsVector;
	cv::Mat descriptorsMatrix;
	cv::Mat mask = cv::Mat();
	orb->detectAndCompute(cvImage, mask, keypointsVector, descriptorsMatrix);

	VisualPointFeatureVector2DPtr featuresVector = new VisualPointFeatureVector2D();
	ClearPoints(*featuresVector);

	for (unsigned pointIndex = 0; pointIndex < keypointsVector.size(); pointIndex++)
		{
		AddPoint(*featuresVector, keypointsVector.at(pointIndex).pt.x,  keypointsVector.at(pointIndex).pt.y);
		for (unsigned componentIndex = 0; componentIndex < descriptorsMatrix.cols; componentIndex++)
			{
			AddDescriptorComponent(*featuresVector, pointIndex, descriptorsMatrix.at<float>(pointIndex, componentIndex));
			}
		}

	flann->sourceFeaturesInput(*featuresVector);
	flann->sinkFeaturesInput(*featuresVector);
	outputWindowName = "Flann Matcher Result";
	}

FlannMatcherTestInterface::~FlannMatcherTestInterface()
	{
	delete(flann);
	}

void FlannMatcherTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "DistanceThreshold", 0.02, 1.00, 0.01);
	AddParameter("GeneralParameters", "MatcherMethod", 4, 6);
	AddParameter("GeneralParameters", "AcceptanceRatio", 0.75, 1, 0.01);
	AddParameter("LocalitySensitiveHashingParameters", "TableNumber", 6, 20);
	AddParameter("LocalitySensitiveHashingParameters", "KeySize", 12, 20);
	AddParameter("LocalitySensitiveHashingParameters", "MultiProbeLevel", 1, 20);
	}

void FlannMatcherTestInterface::DisplayResult()
	{
	const CorrespondenceMap2D& correspondenceMap = flann->matchesOutput();
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	std::vector<cv::KeyPoint> sourceVector, sinkVector;
	std::vector<cv::DMatch> matchesVector;
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
		sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
		sinkPoint = GetSink(correspondenceMap, correspondenceIndex);
		cv::KeyPoint sourceKeypoint(sourcePoint.x, sourcePoint.y, 0.02);
		cv::KeyPoint sinkKeypoint(sinkPoint.x, sinkPoint.y, 0.02);
		sourceVector.push_back(sourceKeypoint);
		sinkVector.push_back(sinkKeypoint);
		cv::DMatch match;
		match.queryIdx = correspondenceIndex;
		match.trainIdx = correspondenceIndex;
		matchesVector.push_back(match);
		}

	cv::Mat outputImage;
	cv::drawMatches(
		cvImage, sourceVector, cvImage, sinkVector, matchesVector,
		outputImage,
		cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
		cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
		);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, outputImage);
	}


int main(int argc, char** argv)
	{
	FlannMatcherTestInterface interface("Flann Matcher", 100, 40);
	interface.Run();
	};

/** @} */
