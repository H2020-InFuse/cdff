/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Icp3D.hpp
 * @date 25/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN executes the ICP algorithms for detecting a 3d model pose in a 3d scene, according to the implementation in PCL library. 
 * 
 * This DFN implementation requires the following parameters:
 * @param maxCorrespondenceDistance, the maximum spatial distance allowed between the transformed point model and a scene point to consider the matching acceptable.
 * @param maximumIterations, this is the maximum number of iteration of the algorithm, after these iterations the best transform (the one with more inliers) is given as output.
 * @param transformationEpsilon, 
 * @param euclideanFitnessEpsilon,
 *
 * @{
 */

#ifndef ICP_3D_HPP
#define ICP_3D_HPP

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
    class Icp3D : public FeaturesMatching3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	Icp3D();
        	~Icp3D();
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
		struct IcpOptionsSet
			{
			double maxCorrespondenceDistance;
			int maximumIterations;
			double transformationEpsilon;
			double euclideanFitnessEpsilon;
			};

		Helpers::ParametersListHelper parametersHelper;
		IcpOptionsSet parameters;
		static const IcpOptionsSet DEFAULT_PARAMETERS;

		PoseWrapper::Transform3DConstPtr ComputeTransform(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		PoseWrapper::Transform3DConstPtr Convert(Eigen::Matrix4f eigenTransform);

		void ValidateParameters();
		void ValidateInputs(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
		void ValidateCloud(Converters::SupportTypes::PointCloudWithFeatures cloud);
    };
}
#endif
/* Icp3D.hpp */
/** @} */
