/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StructureFromMotion.hpp
 * @date 18/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Structure From Motion as implementation of the DPFC for Point Cloud Model Localisation.
 *  
 *
 * @{
 */

#ifndef STRUCTURE_FROM_MOTION_HPP
#define STRUCTURE_FROM_MOTION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StructureFromMotion : public PointCloudModelLocalisationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		StructureFromMotion();
		~StructureFromMotion();
		void process();

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

		dfn_ci::ImageFilteringInterface* filter;
		dfn_ci::FeaturesExtraction2DInterface* featureExtractor;
		dfn_ci::FeaturesDescription2DInterface* optionalFeatureDescriptor;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher;	
		dfn_ci::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		dfn_ci::CamerasTransformEstimationInterface* cameraTransformEstimator;
		dfn_ci::PointCloudReconstruction2DTo3DInterface* reconstructor3D;

		std::vector<FrameWrapper::FrameConstPtr> imagesHistory;
		FrameWrapper::FrameConstPtr pastImage;
		FrameWrapper::FrameConstPtr currentImage;
		FrameWrapper::FrameConstPtr filteredPastImage;
		FrameWrapper::FrameConstPtr filteredCurrentImage;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap;
		MatrixWrapper::Matrix3dConstPtr fundamentalMatrix;
		PoseWrapper::Pose3DConstPtr pastToCurrentCameraTransform;
		PointCloudWrapper::PointCloudConstPtr pointCloud;

		void AssignDfnsAlias();

		void FilterCurrentImage();
		void FilterPastImage();
		void ExtractCurrentFeatures();
		void ExtractPastFeatures();
		void DescribeCurrentFeatures();
		void DescribePastFeatures();
		void MatchCurrentAndPastFeatures();
		bool ComputeFundamentalMatrix();
		bool ComputePastToCurrentTransform();
		void ComputePointCloud();
    };
}
#endif
/* StructureFromMotion.hpp */
/** @} */
