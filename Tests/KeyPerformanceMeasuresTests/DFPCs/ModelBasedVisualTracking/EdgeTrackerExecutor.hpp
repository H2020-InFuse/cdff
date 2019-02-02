/**
 * @author Nassir W. Oumer
 */

#ifndef EDGETRACKEREXECUTOR_HPP
#define EDGETRACKEREXECUTOR_HPP

#include <ModelBasedVisualTracking/ModelBasedVisualTrackingInterface.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/C/RigidBodyState.h>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>

#include <string>
#include <vector>
#include <fstream>

/**
 * This class is used to execute the ModelBasedVisualTracking DFPC
 * on a sequence of input images
 */
class EdgeTrackerExecutor
{
	public:

		EdgeTrackerExecutor();
		~EdgeTrackerExecutor();

		double dtImages;
		bool logGroundTruthError;

		void SetDfpc(
			std::string configurationFilePath, CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc);

		void SetInputFilesPaths(
			std::string inputImagesFolder, std::string inputImagesListFileName,
			std::string inputPosesFolder = std::string(), std::string inputPosesListFileName = std::string());
		void SetOutputFilePath(std::string outputPoseFilePath);

		void initPose(double* guessT0);
		void initVelocity(double* velocity0);

		void ExecuteDfpc();

		void SaveOutputPose(std::ofstream& writer, double* guessT0 = NULL);

	private:

		std::string inputImagesFolder;
		std::string inputImagesListFileName;
		std::vector<std::string> leftImageFileNamesList;
		std::vector<std::string> rightImageFileNamesList;

		FrameWrapper::FrameConstPtr inputLeftFrame;
		FrameWrapper::FrameConstPtr inputRightFrame;
		Converters::MatToFrameConverter frameConverter;

		std::string inputPosesFolder;
		std::string inputPosesListFileName;
		std::vector<std::string> poseFileNamesList;

		asn1SccRigidBodyState outputPose;
		std::string outputPoseFilePath;

		std::string configurationFilePath;

		CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc;

		bool inputImagesWereLoaded;
		bool outputPoseWasLoaded;
		bool dfpcWasLoaded;
		bool dfpcExecuted;
		bool outputSuccess;

		void ConfigureDfpc();

		void LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame);
		void LoadInputImagesList();

		void LoadInputPose(std::string filePath, double* groundTruthT);
		void LoadInputPosesList();

		bool isFileExist(const std::string& name);
		void setState(asn1SccRigidBodyState& state, const double value);

		void ConvertAsnStateToState(asn1SccRigidBodyState& poseState, double* pose, double* velocity = NULL);
		asn1SccRigidBodyState ConvertStateToAsnState(double* pose, double* velocity = NULL);
};

#endif // EDGETRACKEREXECUTOR_HPP
