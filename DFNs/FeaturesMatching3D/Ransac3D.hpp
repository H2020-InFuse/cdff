/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Ransac3D.hpp
 * @date 17/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the RANSAC for matching points between two 3D point clouds.
 *  
 *
 * @{
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN executes the RANSAC algorithms for detecting a 3d model pose in a 3d scene, according to the implementation in PCL library. 
 * 
 * This DFN implementation requires the following parameters:
 * @param similarityThreshold, this is the minimum amount of similarity between two features, as a condition for considering them a possible match.
 * @param inlierFraction, when the model is transformed into the coordinate system of the scene, this is the minimum fraction of model points that has to be found in the scene, as a condition for
 *				accepting the transformation.
 * @param correspondenceRandomness, 
 * @param numberOfSamples, this is the number of random samples that are selected in the intial phase of RANSAC for the construction of a model,
 * @param maximumIterations, this is the maximum number of iteration of the algorithm, after these iterations the best transform (the one with more inliers) is given as output,
 * @param maxCorrespondenceDistance.searchRadius, the maximum spatial distance allowed between the transformed point model and a scene point to consider the matching acceptable.
 *
 * @{
 */

#ifndef RANSAC_3D_HPP
#define RANSAC_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <Pose.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include <SupportTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class Ransac3D : public FeaturesMatching3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	Ransac3D();
        	~Ransac3D();
        	void process();
        	void configure();

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
		struct RansacOptionsSet
			{
			float similarityThreshold;
			float inlierFraction;
			int correspondenceRandomness;
			int numberOfSamples;
			int maximumIterations;
			float maxCorrespondenceDistance;
			};

		Helpers::ParametersListHelper parametersHelper;
		RansacOptionsSet parameters;
		static const RansacOptionsSet DEFAULT_PARAMETERS;

		PoseWrapper::Transform3DConstPtr ComputeTransform(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		PoseWrapper::Transform3DConstPtr Convert(Eigen::Matrix4f eigenTransform);

		void ValidateParameters();
		void ValidateInputs(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		void ValidateCloud(Converters::SupportTypes::PointCloudWithFeatures cloud);
    };
}
#endif
/* Ransac3D.hpp */
/** @} */
