/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PoseWrapper
 * @{
 */

#include "PosesSequence.hpp"
#include <Errors/Assert.hpp>

namespace PoseWrapper
{

using namespace BaseTypesWrapper;

void Copy(const Poses3DSequence& source, Poses3DSequence& destination)
{
	Clear(destination);
	T_UInt32 numberOfPoses = GetNumberOfPoses(source);
	for(T_UInt32 poseIndex = 0; poseIndex < numberOfPoses; poseIndex++)
		{
		AddPose(destination, GetPose(source, poseIndex));
		}
}

Poses3DSequencePtr NewPoses3DSequence()
{
	Poses3DSequencePtr posesSequence = new Poses3DSequence();
	Initialize(*posesSequence);
	return posesSequence;
}

Poses3DSequenceSharedPtr NewSharedPoses3DSequence()
{
	Poses3DSequenceSharedPtr sharedPosesSequence = std::make_shared<Poses3DSequence>();
	Initialize(*sharedPosesSequence);
	return sharedPosesSequence;
}

Poses3DSequenceConstPtr Clone(const Poses3DSequence& source)
{
	Poses3DSequencePtr posesSequence = new Poses3DSequence();
	Copy(source, *posesSequence);
	return posesSequence;
}

Poses3DSequenceSharedPtr SharedClone(const Poses3DSequence& source)
{
	Poses3DSequenceSharedPtr sharedPosesSequence = std::make_shared<Poses3DSequence>();
	Copy(source, *sharedPosesSequence);
	return sharedPosesSequence;
}

void Initialize(Poses3DSequence& posesSequence)
{
	Clear(posesSequence);
}

void Clear(Poses3DSequence& posesSequence)
{
	posesSequence.nCount = 0;
}

void AddPose(Poses3DSequence& posesSequence, const Pose3D& pose)
{
	ASSERT( GetNumberOfPoses(posesSequence) < MAX_POSES_SEQUENCE_LENGTH, "Error, poses sequence limit reached");
	Copy( pose, posesSequence.arr[posesSequence.nCount] );
	posesSequence.nCount++;
}

const Pose3D& GetPose(const Poses3DSequence& posesSequence, BaseTypesWrapper::T_UInt32 poseIndex)
{
	return posesSequence.arr[poseIndex];
}

void GetPose(const Poses3DSequence& posesSequence, BaseTypesWrapper::T_UInt32 poseIndex, Pose3D& pose)
{
	Copy( GetPose(posesSequence, poseIndex), pose);
}

BaseTypesWrapper::T_UInt32 GetNumberOfPoses(const Poses3DSequence& posesSequence)
{
	return posesSequence.nCount;
}

BitStream ConvertToBitStream(const Poses3DSequence& sequence)
	CONVERT_TO_BIT_STREAM(sequence, asn1SccPosesSequence_REQUIRED_BYTES_FOR_ENCODING, asn1SccPosesSequence_Encode)

void ConvertFromBitStream(BitStream bitStream, Poses3DSequence& sequence)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccPosesSequence_REQUIRED_BYTES_FOR_ENCODING, sequence, asn1SccPosesSequence_Decode)

}

/** @} */
