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
 *  This DFN implements the ICP algorithm for matching points between two 3D point clouds.
 *  
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
