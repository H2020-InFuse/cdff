/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup FrameWrapper
 * @{
 */

#include "Frame.hpp"

namespace FrameWrapper
{

using namespace BaseTypesWrapper;
	
	void CopyTransformWithCovariance(const asn1SccTransformWithCovariance& source, asn1SccTransformWithCovariance& destination)
	{
	for(int i=0; i<3; i++) 
		{
		destination.data.translation.arr[i] = source.data.translation.arr[i];
		}
	for(int i=0; i<4; i++) 
		{
		destination.data.orientation.arr[i] = source.data.orientation.arr[i];
		}
	for(int row=0; row<6; row++) 
		{
		for(int column=0; column<6; column++) 
			{
			destination.data.cov.arr[row].arr[column] = source.data.cov.arr[row].arr[column];
			}	
		}
	destination.metadata.msgVersion = source.metadata.msgVersion;
    CopyString(source.metadata.producerId, destination.metadata.producerId);
	for(int i=0; i<7; i++) 
		{
		destination.metadata.dataEstimated.arr[i] = source.metadata.dataEstimated.arr[i];
		}
	CopyString(source.metadata.parentFrameId, destination.metadata.parentFrameId);
	destination.metadata.parentTime = source.metadata.parentTime;
	CopyString(source.metadata.childFrameId, destination.metadata.childFrameId);
	destination.metadata.childTime = source.metadata.childTime;
	}


void Copy(const Frame& source, Frame& destination)
{

	destination.msgVersion = source.msgVersion;
	destination.metadata.msgVersion = source.metadata.msgVersion;
	destination.metadata.timeStamp = source.metadata.timeStamp;
	destination.metadata.receivedTime = source.metadata.receivedTime; 
	destination.metadata.pixelModel = source.metadata.pixelModel;
	destination.metadata.pixelCoeffs.nCount = source.metadata.pixelCoeffs.nCount;
	for(int i=0; i< source.metadata.pixelCoeffs.nCount; i++) 
		{
		destination.metadata.pixelCoeffs.arr[i] = source.metadata.pixelCoeffs.arr[i];
		}

	destination.metadata.errValues.nCount = source.metadata.errValues.nCount;
	for(int i=0; i<3; i++)
		{
		destination.metadata.errValues.arr[i] = source.metadata.errValues.arr[i];
		}

	destination.metadata.attributes.nCount = source.metadata.attributes.nCount;
	for(int i=0; i<5; i++)
		{
		destination.metadata.attributes.arr[i] = source.metadata.attributes.arr[i];
		} 

	destination.metadata.mode = source.metadata.mode;
	destination.metadata.status = source.metadata.status;
	destination.intrinsic.msgVersion = source.intrinsic.msgVersion;
	CopyString(source.intrinsic.sensorId, destination.intrinsic.sensorId);
	for(int row = 0; row < 3; row++) 
		{
		for (int column = 0; column < 3; column++) 
			{
			destination.intrinsic.cameraMatrix.arr[row].arr[column] = source.intrinsic.cameraMatrix.arr[row].arr[column];
			}
		}

	destination.intrinsic.cameraModel = source.intrinsic.cameraModel ;
	destination.intrinsic.distCoeffs.nCount = source.intrinsic.distCoeffs.nCount;
	for(int i=0; i<  source.intrinsic.distCoeffs.nCount; i++) 
		{
		destination.intrinsic.distCoeffs.arr[i] = source.intrinsic.distCoeffs.arr[i];
		}
	destination.extrinsic.msgVersion = source.extrinsic.msgVersion;
	destination.extrinsic.hasFixedTransform = source.extrinsic.hasFixedTransform;
	CopyTransformWithCovariance(source.extrinsic.pose_robotFrame_sensorFrame, destination.extrinsic.pose_robotFrame_sensorFrame);
	CopyTransformWithCovariance(source.extrinsic.pose_fixedFrame_robotFrame, destination.extrinsic.pose_fixedFrame_robotFrame);

    Array3DWrapper::Copy(source.data, destination.data);
}

FramePtr NewFrame()
{
	FramePtr frame = new Frame();
	Initialize(*frame);
	return frame;
}

FrameSharedPtr NewSharedFrame()
{
	FrameSharedPtr sharedFrame = std::make_shared<Frame>();
	Initialize(*sharedFrame);
	return sharedFrame;
}

FrameConstPtr Clone(const Frame& source)
{
	FramePtr frame = new Frame();
	Copy(source, *frame);
	return frame;
}

FrameSharedPtr SharedClone(const Frame& source)
{
	FrameSharedPtr sharedFrame = std::make_shared<Frame>();
	Copy(source, *sharedFrame);
	return sharedFrame;
}

void Initialize(Frame& frame)
{
Array3DWrapper::Initialize(frame.data);

frame.metadata.pixelModel = FramePixelMode::asn1Sccpix_UNDEF;
frame.metadata.mode = FrameMode::asn1Sccmode_UNDEF;
frame.metadata.status = FrameStatus::asn1Sccstatus_EMPTY;

frame.metadata.pixelCoeffs.nCount = 0;
frame.metadata.errValues.nCount = 0;
frame.metadata.attributes.nCount = 0;
frame.intrinsic.sensorId.nCount = 0;
frame.intrinsic.distCoeffs.nCount = 0;
frame.extrinsic.pose_robotFrame_sensorFrame.metadata.producerId.nCount = 0;
frame.extrinsic.pose_robotFrame_sensorFrame.metadata.parentFrameId.nCount = 0;
frame.extrinsic.pose_robotFrame_sensorFrame.metadata.childFrameId.nCount = 0;
frame.extrinsic.pose_fixedFrame_robotFrame.metadata.producerId.nCount = 0;
frame.extrinsic.pose_fixedFrame_robotFrame.metadata.parentFrameId.nCount = 0;
frame.extrinsic.pose_fixedFrame_robotFrame.metadata.childFrameId.nCount = 0;
}

void SetFrameMode(Frame& frame, FrameMode frameMode)
{
	frame.metadata.mode = frameMode;
}

FrameMode GetFrameMode(const Frame& frame)
{
	return frame.metadata.mode;
}

void SetFrameSize(Frame& frame, T_UInt16 width, T_UInt16 height)
{
	frame.data.cols = width;
	frame.data.rows = height;
}

T_UInt16 GetFrameWidth(const Frame& frame)
{
	return frame.data.cols;
}

T_UInt16 GetFrameHeight(const Frame& frame)
{
	return frame.data.rows;
}

void SetFrameStatus(Frame& frame, FrameStatus frameStatus)
{
    frame.metadata.status = frameStatus;
}

FrameStatus GetFrameStatus(const Frame& frame)
{
    return frame.metadata.status;
}

void ClearData(Frame& frame, bool overwrite)
{
	frame.data.data.nCount = 0;
	if (overwrite)
	{
		memset(frame.data.data.arr, 0, Array3DWrapper::MAX_ARRAY3D_BYTE_SIZE);
	}
}

byte GetDataByte(const Frame& frame, int index)
{
	ASSERT_ON_TEST(index < frame.data.data.nCount, "Requesting missing image data");
	return frame.data.data.arr[index];
}

int GetNumberOfDataBytes(const Frame& frame)
{
	return frame.data.data.nCount;
}

BitStream ConvertToBitStream(const Frame& frame)
	CONVERT_TO_BIT_STREAM(frame, asn1SccFrame_REQUIRED_BYTES_FOR_ENCODING, asn1SccFrame_Encode)

void ConvertFromBitStream(BitStream bitStream, Frame& frame)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccFrame_REQUIRED_BYTES_FOR_ENCODING, frame, asn1SccFrame_Decode)

}

/** @} */
