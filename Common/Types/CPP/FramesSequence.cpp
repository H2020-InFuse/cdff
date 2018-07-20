/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup FrameWrapper
 * @{
 */

#include "FramesSequence.hpp"
#include <Errors/Assert.hpp>

namespace FrameWrapper
{

using namespace BaseTypesWrapper;

void Copy(const FramesSequence& source, FramesSequence& destination)
{
	Clear(destination);
	for(T_UInt32 frameIndex = 0; frameIndex < GetNumberOfFrames(destination); frameIndex++)
		{
		AddFrame(destination, GetFrame(source, frameIndex));
		}
}

FramesSequencePtr NewFramesSequence()
{
	FramesSequencePtr framesSequence = new FramesSequence();
	Initialize(*framesSequence);
	return framesSequence;
}

FramesSequenceSharedPtr NewSharedFramesSequence()
{
	FramesSequenceSharedPtr sharedFramesSequence = std::make_shared<FramesSequence>();
	Initialize(*sharedFramesSequence);
	return sharedFramesSequence;
}

FramesSequenceConstPtr Clone(const FramesSequence& source)
{
	FramesSequencePtr framesSequence = new FramesSequence();
	Copy(source, *framesSequence);
	return framesSequence;
}

FramesSequenceSharedPtr SharedClone(const FramesSequence& source)
{
	FramesSequenceSharedPtr sharedFramesSequence = std::make_shared<FramesSequence>();
	Copy(source, *sharedFramesSequence);
	return sharedFramesSequence;
}

void Initialize(FramesSequence& framesSequence)
{
	Clear(framesSequence);
}

void Clear(FramesSequence& framesSequence)
{
	framesSequence.nCount = 0;
}


void AddFrame(FramesSequence& framesSequence, const Frame& frame)
{
	ASSERT( GetNumberOfFrames(framesSequence) < MAX_FRAMES_SEQUENCE_LENGTH, "Error, frames sequence limit reached");
	Copy( frame, framesSequence.arr[framesSequence.nCount] );
	framesSequence.nCount++;
}

const Frame& GetFrame(const FramesSequence& framesSequence, BaseTypesWrapper::T_UInt32 frameIndex)
{
	return framesSequence.arr[frameIndex];
}

void GetFrame(const FramesSequence& framesSequence, BaseTypesWrapper::T_UInt32 frameIndex, Frame& frame)
{
	Copy( GetFrame(framesSequence, frameIndex), frame);
}

BaseTypesWrapper::T_UInt32 GetNumberOfFrames(const FramesSequence& framesSequence)
{
	return framesSequence.nCount;
}

BitStream ConvertToBitStream(const FramesSequence& sequence)
	CONVERT_TO_BIT_STREAM(sequence, asn1SccFramesSequence_REQUIRED_BYTES_FOR_ENCODING, asn1SccFramesSequence_Encode)

void ConvertFromBitStream(BitStream bitStream, FramesSequence& sequence)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccFramesSequence_REQUIRED_BYTES_FOR_ENCODING, sequence, asn1SccFramesSequence_Decode)

}

/** @} */
