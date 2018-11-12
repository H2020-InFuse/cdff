/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EdgeEdgeTrackerExecutor.hpp
 * @date 10/07/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class is used for execution of the DFPC ModelBasedVisualTracking on a sequence of input images
 *
 * @{
 */

#ifndef EDGE_TRACKER_EXECUTOR_HPP
#define EDGE_TRACKER_EXECUTOR_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ModelBasedVisualTracking/ModelBasedVisualTrackingInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/Frame.hpp>
#include <Types/C/RigidBodyState.h>
#include <Types/C/Time.h>
#include <Converters/SupportTypes.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class EdgeTrackerExecutor
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		EdgeTrackerExecutor();
		~EdgeTrackerExecutor();
		double dtImages;
		bool logGroundTruthError;
		void SetDfpc(std::string configurationFilePath, CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc);
		void SetInputFilesPaths(std::string inputImagesFolder, std::string inputImagesListFileName,std::string inputPosesFolder = std::string(), std::string 				inputPosesListFileName = std::string());
		void SetOutputFilePath(std::string outputPoseFilePath);
		void initPose(double* guessT0);
		void initVelocity(double* velocity0);
		void ExecuteDfpc();
		void SaveOutputPose(std::ofstream& writer, double* guessT0 = NULL);

		
		

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		
		std::string configurationFilePath;
		std::string inputImagesFolder;
		std::string inputImagesListFileName;
		std::string outputPoseFilePath;
		
		std::vector<std::string> leftImageFileNamesList;
		std::vector<std::string> rightImageFileNamesList;

		std::string inputPosesFolder;
		std::string inputPosesListFileName;
		std::vector<std::string> poseFileNamesList;

		FrameWrapper::FrameConstPtr inputLeftFrame;
		FrameWrapper::FrameConstPtr inputRightFrame;
		asn1SccRigidBodyState outputPose;
		bool outputSuccess;
		
		Converters::MatToFrameConverter frameConverter;
		CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc;
		
		bool inputImagesWereLoaded;
		bool outputPoseWasLoaded;
		bool dfpcExecuted;
		bool dfpcWasLoaded;
		void LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame);
		void LoadInputPose(std::string filePath, double* groundTruthT);
		void LoadInputImagesList();
		void LoadInputPosesList();
		void LoadOutputPose();
		void ConfigureDfpc();
		void filterMedian(cv::Mat& image, cv::Mat& filteredImage, int apertureSize);
		bool isFileExist(const std::string& name);
		void ConvertAsnStateToState(asn1SccRigidBodyState& poseState, double* pose, double* velocity = NULL);
		asn1SccRigidBodyState ConvertStateToAsnState(double* pose, double* velocity = NULL);
		void setState(asn1SccRigidBodyState& state, const double value);
		
				

		
	};

#endif

/* EdgeTrackerExecutor.hpp */
/** @} */
