/**
 * @author Clément Bazerque
 */

/**
 * Test application for the DFN DisparityImage
 */

/**
 * @addtogroup DFNsTest
 * @{
 */
#include <DisparityImage/DisparityImage.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace CDFF::DFN::DisparityImage;
using namespace FrameWrapper;

class DisparityImageTestInterface : public DFNTestInterface
{
	public:
		DisparityImageTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
		~DisparityImageTestInterface();

	private:
		DisparityImage disparityImage;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;
		std::string outputWindowName;

		void SetupParameters() override;
		void DisplayResult() override;
};

DisparityImageTestInterface::DisparityImageTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
	DFNTestInterface(dfnName, buttonWidth, buttonHeight),
	disparityImage()
{
	SetDFN(&disparityImage);

	// Loads two grayscaled rectified images
	cvLeftImage = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRectLeft.png", cv::IMREAD_GRAYSCALE);
	cvRightImage = cv::imread("../../tests/Data/Images/MinnieStereo/MinnieRectRight.png", cv::IMREAD_GRAYSCALE);

	// Initialise a frame pair and set its metadata
	asn1SccFramePair *framePair = new asn1SccFramePair();
	asn1SccFramePair_Initialize(framePair);
	framePair->msgVersion = frame_Version;
	framePair->baseline = 0.270268442641143;

	// Fill the left image data
	{
		framePair->left.msgVersion = frame_Version;
		framePair->left.metadata.msgVersion = frame_Version;
		framePair->left.metadata.status = asn1Sccstatus_VALID;
		framePair->left.metadata.pixelModel = asn1Sccpix_UNDEF;
		framePair->left.metadata.mode = asn1Sccmode_GRAY;

		framePair->left.intrinsic.msgVersion = frame_Version;
		framePair->left.intrinsic.cameraModel = asn1Scccam_PINHOLE;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[0] = 1069.23438117834;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[0].arr[2] = 956.907250034732;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[1] = 1071.73940921359;
		framePair->left.intrinsic.cameraMatrix.arr[1].arr[2] = 621.662860111553;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
		framePair->left.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

		asn1SccArray3D &imageOnly = framePair->left.data;
		imageOnly.msgVersion = array3D_Version;
		imageOnly.rows = static_cast<asn1SccT_UInt32>(cvLeftImage.rows);
		imageOnly.cols = static_cast<asn1SccT_UInt32>(cvLeftImage.cols);
		imageOnly.channels = static_cast<asn1SccT_UInt32>(cvLeftImage.channels());
		imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvLeftImage.depth());
		imageOnly.rowSize = cvLeftImage.step[0];
		imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
		memcpy(imageOnly.data.arr, cvLeftImage.data, static_cast<size_t>(imageOnly.data.nCount));
	}

	// Fill the right image data
	{
		framePair->right.msgVersion = frame_Version;
		framePair->right.metadata.msgVersion = frame_Version;
		framePair->right.metadata.status = asn1Sccstatus_VALID;
		framePair->right.metadata.pixelModel = asn1Sccpix_UNDEF;
		framePair->right.metadata.mode = asn1Sccmode_GRAY;

		framePair->right.intrinsic.msgVersion = frame_Version;
		framePair->right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[0] = 1066.44771376037;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[0].arr[2] = 943.937535155249;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[1] = 1069.28469804725;
		framePair->right.intrinsic.cameraMatrix.arr[1].arr[2] = 617.141031157546;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
		framePair->right.intrinsic.cameraMatrix.arr[2].arr[2] = 1;
		asn1SccArray3D &imageOnly = framePair->right.data;
		imageOnly.msgVersion = array3D_Version;
		imageOnly.rows = static_cast<asn1SccT_UInt32>(cvRightImage.rows);
		imageOnly.cols = static_cast<asn1SccT_UInt32>(cvRightImage.cols);
		imageOnly.channels = static_cast<asn1SccT_UInt32>(cvRightImage.channels());
		imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvRightImage.depth());
		imageOnly.rowSize = cvRightImage.step[0];
		imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
		memcpy(imageOnly.data.arr, cvRightImage.data, static_cast<size_t>(imageOnly.data.nCount));
	}

	// Set the DFN input with this image pair
	disparityImage.framePairInput(*framePair);

	outputWindowName = "Disparity Image Result";

	delete (framePair);
}

DisparityImageTestInterface::~DisparityImageTestInterface()
{

}

void DisparityImageTestInterface::SetupParameters()
{
    AddParameter("stereoMatcherParams", "algorithm", 1, 2);
    AddParameter("stereoMatcherParams", "minDisparity", 0, 160);
    AddParameter("stereoMatcherParams", "numDisparities", 192, 640, 16);
    AddParameter("stereoMatcherParams", "blockSize", 5, 39);
    AddParameter("stereoMatcherParams", "speckleWindowSize", 0, 200);
    AddParameter("stereoMatcherParams", "speckleRange", 0, 10);
    AddSignedParameter("stereoMatcherParams", "disp12MaxDiff", -1, 50, 1);
    AddSignedParameter("stereoMatcherParams", "preFilterCap", 31, 63, 1, 1);
    AddParameter("stereoMatcherParams", "uniquenessRatio", 10, 100);

    AddParameter("stereoBMParams", "preFilterType", 0, 2);
    AddParameter("stereoBMParams", "preFilterSize", 9, 255);
    AddParameter("stereoBMParams", "textureThreshold", 10, 100);

    AddParameter("stereoSGBMParams", "P1", 0, 100);
    AddParameter("stereoSGBMParams", "P2", 0, 100);
    AddParameter("stereoSGBMParams", "mode", 0, 3);

#if WITH_XIMGPROC
    AddParameter("filterParams", "useFilter", 0, 1);
    AddParameter("filterParams", "useConfidence", 0, 1);
    AddParameter("filterParams", "depthDiscontinuityRadius", 0, 100);
    AddParameter("filterParams", "lambda", 8000, 20000);
    AddParameter("filterParams", "lrcThresh", 24, 100);
    AddParameter("filterParams", "sigmaColor", 0.8, 2, 0.1);
#endif
}

void DisparityImageTestInterface::DisplayResult()
{
	// Fetch the resulting disparity image
	asn1SccFrame* res =  new asn1SccFrame();
    *res = disparityImage.disparityOutput();

	PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
	PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());

	// Convert the disparity image as a cv::Mat for display
	cv::Mat disparity = cv::Mat(res->data.rows, res->data.cols, CV_MAKETYPE((int)(res->data.depth), res->data.channels), res->data.data.arr, res->data.rowSize);

	// Apply a colormap
	cv::Mat dispColor;
	double min,	max;
	cv::minMaxLoc(disparity, &min, &max);
	disparity.convertTo(disparity, CV_8U, 255 / (max - min), -255.0 * min / (max - min));
	cv::Mat mask = disparity > 0;
	cv::applyColorMap(disparity, disparity, 2);
	disparity.copyTo(dispColor, mask);

	// Display the disparity
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, dispColor);
	cv::resizeWindow(outputWindowName, dispColor.cols, dispColor.rows);
	cv::waitKey(500);

	delete(res);
}

int main(int argc, char** argv)
{
	DisparityImageTestInterface* interface = new DisparityImageTestInterface("DisparityImage", 100, 40);
	interface->Run();
	delete(interface);
};

/** @} */
