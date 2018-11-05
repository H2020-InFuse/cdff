/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup FrameWrapper
 *
 * Wrapper for ASN.1 Frames Sequence types
 *
 * @{
 */

#ifndef FRAMES_SEQUENCE_HPP
#define FRAMES_SEQUENCE_HPP

#include <Types/C/Sequences.h>

#include "BaseTypes.hpp"
#include "Frame.hpp"
#include <stdlib.h>
#include <memory>

namespace FrameWrapper
{

// Types
typedef asn1SccFramesSequence FramesSequence;

// Global constant variables
const int MAX_FRAMES_SEQUENCE_LENGTH = maxFramesSequenceLength;

// Pointer types

typedef FramesSequence* FramesSequencePtr;
typedef FramesSequence const* FramesSequenceConstPtr;
typedef std::shared_ptr<FramesSequence> FramesSequenceSharedPtr;
typedef std::shared_ptr<const FramesSequence> FramesSequenceSharedConstPtr;

// Functions

void Copy(const FramesSequence& source, FramesSequence& destination);
FramesSequencePtr NewFramesSequence();
FramesSequenceSharedPtr NewSharedFramesSequence();
FramesSequenceConstPtr Clone(const FramesSequence& source);
FramesSequenceSharedPtr SharedClone(const FramesSequence& source);
void Initialize(FramesSequence& framesSequence);

void Clear(FramesSequence& framesSequence);
void AddFrame(FramesSequence& framesSequence, const Frame& frame);
const Frame& GetFrame(const FramesSequence& framesSequence, BaseTypesWrapper::T_UInt32 frameIndex);
void GetFrame(const FramesSequence& framesSequence, BaseTypesWrapper::T_UInt32 frameIndex, Frame& frame);
BaseTypesWrapper::T_UInt32 GetNumberOfFrames(const FramesSequence& framesSequence);

BitStream ConvertToBitStream(const FramesSequence& sequence);
void ConvertFromBitStream(BitStream bitStream, FramesSequence& sequence);
}

#endif // FRAMES_SEQUENCE_HPP

/** @} */
