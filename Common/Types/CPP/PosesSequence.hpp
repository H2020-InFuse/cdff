/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PosesWrapper
 *
 * Wrapper for ASN.1 Poses Sequence types
 *
 * @{
 */

#ifndef POSES_SEQUENCE_HPP
#define POSES_SEQUENCE_HPP

#include <Types/C/Sequences.h>

#include "BaseTypes.hpp"
#include "Pose.hpp"
#include <stdlib.h>
#include <memory>

namespace PoseWrapper
{

// Types
typedef asn1SccPosesSequence Poses3DSequence;

// Global constant variables
const int MAX_POSES_SEQUENCE_LENGTH = maxPosesSequenceLength;

// Pointer types

typedef Poses3DSequence* Poses3DSequencePtr;
typedef Poses3DSequence const* Poses3DSequenceConstPtr;
typedef std::shared_ptr<Poses3DSequence> Poses3DSequenceSharedPtr;
typedef std::shared_ptr<const Poses3DSequence> Poses3DSequenceSharedConstPtr;

// Functions

void Copy(const Poses3DSequence& source, Poses3DSequence& destination);
Poses3DSequencePtr NewPoses3DSequence();
Poses3DSequenceSharedPtr NewSharedPoses3DSequence();
Poses3DSequenceConstPtr Clone(const Poses3DSequence& source);
Poses3DSequenceSharedPtr SharedClone(const Poses3DSequence& source);
void Initialize(Poses3DSequence& posesSequence);

void Clear(Poses3DSequence& posesSequence);
void AddPose(Poses3DSequence& posesSequence, const Pose3D& pose);
const Pose3D& GetPose(const Poses3DSequence& posesSequence, BaseTypesWrapper::T_UInt32 poseIndex);
void GetPose(const Poses3DSequence& posesSequence, BaseTypesWrapper::T_UInt32 frameIndex, Pose3D& pose);
BaseTypesWrapper::T_UInt32 GetNumberOfPoses(const Poses3DSequence& posesSequence);

BitStream ConvertToBitStream(const Poses3DSequence& sequence);
void ConvertFromBitStream(BitStream bitStream, Poses3DSequence& sequence);
}

#endif // POSES_SEQUENCE_HPP

/** @} */
