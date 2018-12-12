/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PositionTester.hpp
 * @date 19/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.10 of deliverable 5.5.
 * "Point cloud should be located and oriented within 10% of the size and relative orientation of the scene."
 * Requirement 4.1.1.10 lists two more properties, but they will not be tested here as they relate to the reconstruction of a point clouds, which is a problem this DFN does not deal with.
 *
 * @{
 */

#ifndef POSITION_TESTER_HPP
#define POSITION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <BundleAdjustment/BundleAdjustmentInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/CorrespondenceMaps2DSequence.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Converters/MatToCorrespondenceMaps2DSequenceConverter.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <boost/make_shared.hpp>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PositionTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		PositionTester(std::string configurationFilePath, CDFF::DFN::BundleAdjustmentInterface* dfn);
		~PositionTester();

		void SetFilesPaths(const std::string& inputCorrespondenceFilePath, const std::string& positionReferenceFilePath);
		void ExecuteDfn();
		bool ArePositionsCloseToReference(float relativeLocationError, float relativeOrientationError, float modelSize);

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
		std::string inputCorrespondenceFilePath;
		std::string positionReferenceFilePath;

		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputCorrespondenceMapsSequence;
		PoseWrapper::Poses3DSequencePtr inputPoseReferenceSequence;
		PoseWrapper::Poses3DSequencePtr outputPosesSequence;
		bool bundleAdjustmentSuccess;

		Converters::MatToCorrespondenceMaps2DSequenceConverter correspondenceConverter;
		CDFF::DFN::BundleAdjustmentInterface* dfn;

		bool correspondencesWereLoaded;
		bool positionReferencesWereLoaded;

		void LoadCorrespondences();
		void LoadPositionReferences();
		void ConfigureDfn();

		float ComputeLocationError();
		float ComputeLocationError(float poseIndex);
		float ComputeOrientationError(float modelSize);
		float ComputeOrientationError(float poseIndex, float modelSize);
	};

#endif

/* PositionTester.hpp */
/** @} */
