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
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<double>("GeneralParameters", "Threshold", parameters.threshold, DEFAULT_PARAMETERS.threshold);
	configurationFilePath = "";
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
	Eigen::Vector3d contact_force(
		inArmEndEffectorWrench.force.arr[0],
		inArmEndEffectorWrench.force.arr[1],
		inArmEndEffectorWrench.force.arr[2]
		);

	if( contact_force.norm() > parameters.threshold )
	{
		Eigen::Vector3d effector_position(inArmEndEffectorPose.pos.arr[0], inArmEndEffectorPose.pos.arr[1], inArmEndEffectorPose.pos.arr[2]);
		Eigen::Quaterniond quaternion2(inArmBasePose.orient.arr[0], inArmBasePose.orient.arr[1], inArmBasePose.orient.arr[2], inArmBasePose.orient.arr[3]);
		quaternion2.normalize();
		Eigen::Matrix3d rotation = quaternion2.toRotationMatrix();
		Eigen::Vector3d position(inArmBasePose.pos.arr[0], inArmBasePose.pos.arr[1], inArmBasePose.pos.arr[2]);
		
		effector_position += position;
		effector_position = rotation * effector_position;

		PointCloudWrapper::AddPoint(outPointCloud, effector_position.x(), effector_position.y(), effector_position.z());
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
