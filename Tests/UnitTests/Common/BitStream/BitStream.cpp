/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file BitStream.cpp
 * @date 26/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from types to bitstreams and viceversa.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <Frame.hpp>
#include <BaseTypes.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Matrix.hpp>
#include <CorrespondenceMap2D.hpp>
#include <CorrespondenceMap3D.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <CorrespondenceMaps2DSequence.hpp>
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;

TEST_CASE( "Frame To BitStream", "[FrameToBitStream]" )
	{
	FramePtr inputFrame = NewFrame();
	SetFrameSize(*inputFrame, 10, 10);
	for(unsigned short rowIndex = 0; rowIndex < 10; rowIndex++)
		{
		for (unsigned short columnIndex = 0; columnIndex < 10; columnIndex++)
			{
			AddDataByte(*inputFrame, rowIndex * 10 + columnIndex);
			}
		}
	BitStream bitStream = ConvertToBitStream(*inputFrame);
	
	FramePtr outputFrame = NewFrame();
	ConvertFromBitStream(bitStream, *outputFrame);

	REQUIRE( GetFrameWidth(*inputFrame) == GetFrameWidth(*outputFrame));
	REQUIRE( GetFrameHeight(*inputFrame) == GetFrameHeight(*outputFrame));
	for(unsigned short rowIndex = 0; rowIndex < 10; rowIndex++)
		{
		for (unsigned short columnIndex = 0; columnIndex < 10; columnIndex++)
			{
			REQUIRE ( GetDataByte(*inputFrame, rowIndex * 10 + columnIndex) == GetDataByte(*outputFrame, rowIndex * 10 + columnIndex) );
			}
		}

	delete(inputFrame);
	delete(outputFrame);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 


TEST_CASE( "PointCloud To BitStream", "[PointCloudToBitStream]" )
	{
	PointCloudPtr inputPointCloud = NewPointCloud();
	AddPoint(*inputPointCloud, 0, 2, 1);
	AddPoint(*inputPointCloud, 0.1, 2.2, 1.3);
	BitStream bitStream = ConvertToBitStream(*inputPointCloud);
	
	PointCloudPtr outputPointCloud = NewPointCloud();
	ConvertFromBitStream(bitStream, *outputPointCloud);

	REQUIRE( GetNumberOfPoints(*inputPointCloud) == GetNumberOfPoints(*outputPointCloud));
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*outputPointCloud); pointIndex++)
		{
		REQUIRE ( GetXCoordinate(*inputPointCloud, pointIndex) == GetXCoordinate(*outputPointCloud, pointIndex) );
		REQUIRE ( GetYCoordinate(*inputPointCloud, pointIndex) == GetYCoordinate(*outputPointCloud, pointIndex) );
		REQUIRE ( GetYCoordinate(*inputPointCloud, pointIndex) == GetZCoordinate(*outputPointCloud, pointIndex) );
		}

	delete(inputPointCloud);
	delete(outputPointCloud);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "Pose3D To BitStream", "[Pose3DToBitStream]" )
	{
	Pose3DPtr inputPose = NewPose3D();
	SetPosition(*inputPose, 0.1, 1.1, 2.2);
	SetOrientation(*inputPose, 0, 1, 0, 0);
	BitStream bitStream = ConvertToBitStream(*inputPose);
	
	Pose3DPtr outputPose = NewPose3D();
	ConvertFromBitStream(bitStream, *outputPose);

	REQUIRE( GetXPosition(*inputPose) == GetXPosition(*outputPose) );
	REQUIRE( GetYPosition(*inputPose) == GetYPosition(*outputPose) );
	REQUIRE( GetZPosition(*inputPose) == GetZPosition(*outputPose) );
	REQUIRE( GetXOrientation(*inputPose) == GetXOrientation(*outputPose) );
	REQUIRE( GetYOrientation(*inputPose) == GetYOrientation(*outputPose) );
	REQUIRE( GetZOrientation(*inputPose) == GetZOrientation(*outputPose) );
	REQUIRE( GetWOrientation(*inputPose) == GetWOrientation(*outputPose) );

	delete(inputPose);
	delete(outputPose);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	}

TEST_CASE( "Pose2D To BitStream", "[Pose2DToBitStream]" )
	{
	Pose2DPtr inputPose = NewPose2D();
	SetPosition(*inputPose, 0.1, 1.1);
	SetOrientation(*inputPose, 2.2);
	BitStream bitStream = ConvertToBitStream(*inputPose);
	
	Pose2DPtr outputPose = NewPose2D();
	ConvertFromBitStream(bitStream, *outputPose);

	REQUIRE( GetXPosition(*inputPose) == GetXPosition(*outputPose) );
	REQUIRE( GetYPosition(*inputPose) == GetYPosition(*outputPose) );
	REQUIRE( GetOrientation(*inputPose) == GetOrientation(*outputPose) );

	delete(inputPose);
	delete(outputPose);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "Matrix3d To BitStream", "[Matrix3dToBitStream]" )
	{
	Matrix3dPtr inputMatrix = NewMatrix3d();
	for(int rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			SetElement(*inputMatrix, rowIndex, columnIndex, columnIndex* 1000 + rowIndex);
			}
		}
	BitStream bitStream = ConvertToBitStream(*inputMatrix);
	
	Matrix3dPtr outputMatrix = NewMatrix3d();
	ConvertFromBitStream(bitStream, *outputMatrix);

	for(int rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			REQUIRE( GetElement(*inputMatrix, rowIndex, columnIndex) == GetElement(*outputMatrix, rowIndex, columnIndex) );
			}
		}

	delete(inputMatrix);
	delete(outputMatrix);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	}

TEST_CASE( "CorrespondenceMap2D To BitStream", "[CorrespondenceMap2DToBitStream]" )
	{
	CorrespondenceMap2DPtr inputMap = NewCorrespondenceMap2D();
	for(int correspondenceIndex = 0; correspondenceIndex < 3; correspondenceIndex++)
		{
		Point2D source, sink;
		source.x = correspondenceIndex;
		source.y = correspondenceIndex + 10;
		sink.x = correspondenceIndex + 100;
		sink.y = correspondenceIndex + 1000;
		AddCorrespondence(*inputMap, source, sink, 1);
		}
	BitStream bitStream = ConvertToBitStream(*inputMap);
	
	CorrespondenceMap2DPtr outputMap = NewCorrespondenceMap2D();
	ConvertFromBitStream(bitStream, *outputMap);

	REQUIRE( GetNumberOfCorrespondences(*inputMap) == GetNumberOfCorrespondences(*outputMap) );
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*inputMap); correspondenceIndex++)
		{
		REQUIRE( GetSource(*inputMap, correspondenceIndex).x == GetSource(*outputMap, correspondenceIndex).x) ;
		REQUIRE( GetSource(*inputMap, correspondenceIndex).y == GetSource(*outputMap, correspondenceIndex).y) ;
		REQUIRE( GetSink(*inputMap, correspondenceIndex).x == GetSink(*outputMap, correspondenceIndex).x) ;
		REQUIRE( GetSink(*inputMap, correspondenceIndex).y == GetSink(*outputMap, correspondenceIndex).y) ;
		}

	delete(inputMap);
	delete(outputMap);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "CorrespondenceMap3D To BitStream", "[CorrespondenceMap3DToBitStream]" )
	{
	CorrespondenceMap3DPtr inputMap = NewCorrespondenceMap3D();
	for(int correspondenceIndex = 0; correspondenceIndex < 3; correspondenceIndex++)
		{
		Point3D source, sink;
		source.x = correspondenceIndex;
		source.y = correspondenceIndex + 10;
		source.z = correspondenceIndex + 50;
		sink.x = correspondenceIndex + 100;
		sink.y = correspondenceIndex + 1000;
		sink.z = correspondenceIndex + 5000;
		AddCorrespondence(*inputMap, source, sink, 1);
		}
	BitStream bitStream = ConvertToBitStream(*inputMap);
	
	CorrespondenceMap3DPtr outputMap = NewCorrespondenceMap3D();
	ConvertFromBitStream(bitStream, *outputMap);

	REQUIRE( GetNumberOfCorrespondences(*inputMap) == GetNumberOfCorrespondences(*outputMap) );
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*inputMap); correspondenceIndex++)
		{
		REQUIRE( GetSource(*inputMap, correspondenceIndex).x == GetSource(*outputMap, correspondenceIndex).x) ;
		REQUIRE( GetSource(*inputMap, correspondenceIndex).y == GetSource(*outputMap, correspondenceIndex).y) ;
		REQUIRE( GetSource(*inputMap, correspondenceIndex).z == GetSource(*outputMap, correspondenceIndex).z) ;
		REQUIRE( GetSink(*inputMap, correspondenceIndex).x == GetSink(*outputMap, correspondenceIndex).x) ;
		REQUIRE( GetSink(*inputMap, correspondenceIndex).y == GetSink(*outputMap, correspondenceIndex).y) ;
		REQUIRE( GetSink(*inputMap, correspondenceIndex).z == GetSink(*outputMap, correspondenceIndex).z) ;
		}

	delete(inputMap);
	delete(outputMap);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "VisualPointFeatureVector2D To BitStream", "[VisualPointFeatureVector2DToBitStream]" )
	{
	VisualPointFeatureVector2DPtr inputVector = NewVisualPointFeatureVector2D();
	for(int featureIndex = 0; featureIndex < 3; featureIndex++)
		{
		AddPoint(*inputVector, featureIndex, featureIndex+10);
		AddDescriptorComponent(*inputVector, featureIndex, 70*featureIndex);
		AddDescriptorComponent(*inputVector, featureIndex, 140*featureIndex);
		}
	BitStream bitStream = ConvertToBitStream(*inputVector);
	
	VisualPointFeatureVector2DPtr outputVector = NewVisualPointFeatureVector2D();
	ConvertFromBitStream(bitStream, *outputVector);

	REQUIRE( GetNumberOfPoints(*inputVector) == GetNumberOfPoints(*outputVector) );
	for(int featureIndex = 0; featureIndex < GetNumberOfPoints(*inputVector); featureIndex++)
		{
		REQUIRE( GetXCoordinate(*inputVector, featureIndex) == GetXCoordinate(*outputVector, featureIndex) ) ;
		REQUIRE( GetYCoordinate(*inputVector, featureIndex) == GetYCoordinate(*outputVector, featureIndex) ) ;
		REQUIRE( GetNumberOfDescriptorComponents(*inputVector, featureIndex) == GetNumberOfDescriptorComponents(*outputVector, featureIndex) ) ;
		for(int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(*inputVector, featureIndex); componentIndex++)
			{
			REQUIRE( GetDescriptorComponent(*inputVector, featureIndex, componentIndex) == GetDescriptorComponent(*outputVector, featureIndex, componentIndex) ) ;
			}
		}

	delete(inputVector);
	delete(outputVector);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	}

TEST_CASE( "VisualPointFeatureVector3D To BitStream", "[VisualPointFeatureVector3DToBitStream]" )
	{
	VisualPointFeatureVector3DPtr inputVector = NewVisualPointFeatureVector3D();
	for(int featureIndex = 0; featureIndex < 3; featureIndex++)
		{
		AddPoint(*inputVector, featureIndex, featureIndex+10, featureIndex+100);
		AddDescriptorComponent(*inputVector, featureIndex, 170*featureIndex);
		AddDescriptorComponent(*inputVector, featureIndex, 1140*featureIndex);
		}
	BitStream bitStream = ConvertToBitStream(*inputVector);
	
	VisualPointFeatureVector3DPtr outputVector = NewVisualPointFeatureVector3D();
	ConvertFromBitStream(bitStream, *outputVector);

	REQUIRE( GetNumberOfPoints(*inputVector) == GetNumberOfPoints(*outputVector) );
	for(int featureIndex = 0; featureIndex < GetNumberOfPoints(*inputVector); featureIndex++)
		{
		REQUIRE( GetXCoordinate(*inputVector, featureIndex) == GetXCoordinate(*outputVector, featureIndex) ) ;
		REQUIRE( GetYCoordinate(*inputVector, featureIndex) == GetYCoordinate(*outputVector, featureIndex) ) ;
		REQUIRE( GetZCoordinate(*inputVector, featureIndex) == GetZCoordinate(*outputVector, featureIndex) ) ;
		REQUIRE( GetNumberOfDescriptorComponents(*inputVector, featureIndex) == GetNumberOfDescriptorComponents(*outputVector, featureIndex) ) ;
		for(int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(*inputVector, featureIndex); componentIndex++)
			{
			REQUIRE( GetDescriptorComponent(*inputVector, featureIndex, componentIndex) == GetDescriptorComponent(*outputVector, featureIndex, componentIndex) ) ;
			}
		}

	delete(inputVector);
	delete(outputVector);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "CorrespondenceMaps2DSequence To BitStream", "[CorrespondenceMaps2DSequenceToBitStream]" )
	{
	CorrespondenceMaps2DSequencePtr inputSequence = NewCorrespondenceMaps2DSequence();
	for(int mapIndex = 0; mapIndex < 3; mapIndex++)
		{
		CorrespondenceMap2DPtr inputMap = NewCorrespondenceMap2D();
		for(int correspondenceIndex = 0; correspondenceIndex < 3; correspondenceIndex++)
			{
			Point2D source, sink;
			source.x = correspondenceIndex;
			source.y = correspondenceIndex + 10;
			sink.x = correspondenceIndex + 100;
			sink.y = correspondenceIndex + 1000;
			AddCorrespondence(*inputMap, source, sink, 1);
			}
		AddCorrespondenceMap(*inputSequence, *inputMap);
		delete(inputMap);
		}
	BitStream bitStream = ConvertToBitStream(*inputSequence);
	
	CorrespondenceMaps2DSequencePtr outputSequence = NewCorrespondenceMaps2DSequence();
	ConvertFromBitStream(bitStream, *outputSequence);

	REQUIRE( GetNumberOfCorrespondenceMaps(*inputSequence) == GetNumberOfCorrespondenceMaps(*outputSequence) );
	for(unsigned mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(*inputSequence); mapIndex++)
		{
		const CorrespondenceMap2D& inputMap = GetCorrespondenceMap(*inputSequence, mapIndex);
		const CorrespondenceMap2D& outputMap = GetCorrespondenceMap(*outputSequence, mapIndex);
		REQUIRE( GetNumberOfCorrespondences(inputMap) == GetNumberOfCorrespondences(outputMap) );
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(inputMap); correspondenceIndex++)
			{
			REQUIRE( GetSource(inputMap, correspondenceIndex).x == GetSource(outputMap, correspondenceIndex).x) ;
			REQUIRE( GetSource(inputMap, correspondenceIndex).y == GetSource(outputMap, correspondenceIndex).y) ;
			REQUIRE( GetSink(inputMap, correspondenceIndex).x == GetSink(outputMap, correspondenceIndex).x) ;
			REQUIRE( GetSink(inputMap, correspondenceIndex).y == GetSink(outputMap, correspondenceIndex).y) ;
			}
		}

	delete(inputSequence);
	delete(outputSequence);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	}


TEST_CASE( "FramesSequence To BitStream", "[FramesSequenceToBitStream]" )
	{
	FramesSequencePtr inputSequence = NewFramesSequence();
	for(int frameIndex = 0; frameIndex < 3; frameIndex++)
		{	
		FramePtr inputFrame = NewFrame();
		SetFrameSize(*inputFrame, 10, 10);
		for(unsigned short rowIndex = 0; rowIndex < 10; rowIndex++)
			{
			for (unsigned short columnIndex = 0; columnIndex < 10; columnIndex++)
				{
				AddDataByte(*inputFrame, rowIndex * 10 + columnIndex);
				}
			}
		AddFrame(*inputSequence, *inputFrame);
		delete(inputFrame);
		}
	BitStream bitStream = ConvertToBitStream(*inputSequence);
	
	FramesSequencePtr outputSequence = NewFramesSequence();
	ConvertFromBitStream(bitStream, *outputSequence);

	REQUIRE( GetNumberOfFrames(*inputSequence) == GetNumberOfFrames(*outputSequence) );
	for(unsigned frameIndex = 0; frameIndex < GetNumberOfFrames(*inputSequence); frameIndex++)
		{
		const Frame& inputFrame = GetFrame(*inputSequence, frameIndex);
		const Frame& outputFrame = GetFrame(*outputSequence, frameIndex);
		REQUIRE( GetFrameWidth(inputFrame) == GetFrameWidth(outputFrame));
		REQUIRE( GetFrameHeight(inputFrame) == GetFrameHeight(outputFrame));
		for(unsigned short rowIndex = 0; rowIndex < 10; rowIndex++)
			{
			for (unsigned short columnIndex = 0; columnIndex < 10; columnIndex++)
				{
				REQUIRE ( GetDataByte(inputFrame, rowIndex * 10 + columnIndex) == GetDataByte(outputFrame, rowIndex * 10 + columnIndex) );
				}
			}
		}

	delete(inputSequence);
	delete(outputSequence);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	} 

TEST_CASE( "Poses3DSequence To BitStream", "[Poses3DSequenceToBitStream]" )
	{
	Poses3DSequencePtr inputSequence = NewPoses3DSequence();
	for(int poseIndex = 0; poseIndex < 3; poseIndex++)
		{
		Pose3DPtr inputPose = NewPose3D();
		SetPosition(*inputPose, 0.1, 1.1, 2.2);
		SetOrientation(*inputPose, 0, 1, 0, 0);
		AddPose(*inputSequence, *inputPose);
		delete(inputPose);
		}
	BitStream bitStream = ConvertToBitStream(*inputSequence);
	
	Poses3DSequencePtr outputSequence = NewPoses3DSequence();
	ConvertFromBitStream(bitStream, *outputSequence);

	REQUIRE( GetNumberOfPoses(*inputSequence) == GetNumberOfPoses(*outputSequence) );
	for(unsigned poseIndex = 0; poseIndex < GetNumberOfPoses(*inputSequence); poseIndex++)	
		{
		const Pose3D& inputPose = GetPose(*inputSequence, poseIndex);
		const Pose3D& outputPose = GetPose(*outputSequence, poseIndex);
		REQUIRE( GetXPosition(inputPose) == GetXPosition(outputPose) );
		REQUIRE( GetYPosition(inputPose) == GetYPosition(outputPose) );
		REQUIRE( GetZPosition(inputPose) == GetZPosition(outputPose) );
		REQUIRE( GetXOrientation(inputPose) == GetXOrientation(outputPose) );
		REQUIRE( GetYOrientation(inputPose) == GetYOrientation(outputPose) );
		REQUIRE( GetZOrientation(inputPose) == GetZOrientation(outputPose) );
		REQUIRE( GetWOrientation(inputPose) == GetWOrientation(outputPose) );
		}

	delete(inputSequence);
	delete(outputSequence);
	BitStreamAllocator::DeallocateBitStream(bitStream);
	}
