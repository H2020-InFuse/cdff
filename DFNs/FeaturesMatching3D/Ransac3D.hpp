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

		RansacOptionsSet parameters;
		static const RansacOptionsSet DEFAULT_PARAMETERS;

		PoseWrapper::Transform3DConstPtr ComputeTransform(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		PoseWrapper::Transform3DConstPtr Convert(Eigen::Matrix4f eigenTransform);

		void ValidateParameters();
		void ValidateInputs(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		void ValidateCloud(Converters::SupportTypes::PointCloudWithFeatures cloud);
		
		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* Ransac3D.hpp */
/** @} */
