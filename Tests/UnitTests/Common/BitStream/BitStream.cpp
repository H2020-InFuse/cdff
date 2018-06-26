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
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace BaseTypesWrapper;

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
