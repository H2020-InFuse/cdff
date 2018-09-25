/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "ThresholdForce.hpp"

#include <Eigen/Dense>

#include "Errors/Assert.hpp"
#include "Types/CPP/PointCloud.hpp"
#include "Validators/Number.hpp"

namespace CDFF
{
namespace DFN
{
namespace ForceMeshGenerator
{

//=====================================================================================================================
ThresholdForce::ThresholdForce()
{
    parametersHelper.AddParameter<double>("GeneralParameters", "Threshold", parameters.threshold, DEFAULT_PARAMETERS.threshold);
	configurationFilePath = "";

	outPointCloud.reset(new asn1SccPointcloud);
	PointCloudWrapper::Initialize(*outPointCloud);
}

//=====================================================================================================================
const ThresholdForce::ThresholdForceOptionsSet ThresholdForce::DEFAULT_PARAMETERS =
{
    /*.threshold =*/ 0.5
};

//=====================================================================================================================
void ThresholdForce::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void ThresholdForce::process()
{
	Eigen::Affine3d base_to_ee_tf =
		Eigen::Translation3d(
				inArmBasePose.pos.arr[0],
				inArmBasePose.pos.arr[1],
				inArmBasePose.pos.arr[2])
		* Eigen::Quaterniond(
				inArmBasePose.orient.arr[0],
				inArmBasePose.orient.arr[1],
				inArmBasePose.orient.arr[2],
				inArmBasePose.orient.arr[3]);

	Eigen::Vector3d contact_force(
		inArmEndEffectorWrench.force.arr[0],
		inArmEndEffectorWrench.force.arr[1],
		inArmEndEffectorWrench.force.arr[2]
		);

	if( contact_force.norm() > parameters.threshold )
	{
		Eigen::Vector3d ee_position =
			base_to_ee_tf.inverse() * Eigen::Vector3d(
				inArmEndEffectorPose.pos.arr[0],
				inArmEndEffectorPose.pos.arr[1],
				inArmEndEffectorPose.pos.arr[2]
			);

		PointCloudWrapper::AddPoint(*outPointCloud, ee_position.x(), ee_position.y(), ee_position.z());
	}
}

//=====================================================================================================================
void ThresholdForce::ValidateParameters()
{
    Validators::Number::IsPositive(parameters.threshold);
}


}
}
}

/** @} */
