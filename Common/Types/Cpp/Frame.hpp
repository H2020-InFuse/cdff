/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Frame.hpp
 * @date 05/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * C++ wrapper for the Frame ASN.1 type
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
namespace CTypes {
#include <Frame.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>

#ifndef FRAME_CPP
#define FRAME_CPP

namespace CppTypes 
{


/* --------------------------------------------------------------------------
 *
 * Cpp typedef definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Frame_mode_t FrameMode;
typedef CTypes::Frame_status_t FrameStatus;
typedef CTypes::Frame_size_t FrameSize;
typedef CTypes::Frame_attrib_t FrameAttribute;
typedef CTypes::Frame_image FrameImage;



/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Frame
	{

	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		typedef std::shared_ptr<Frame> Ptr;
		typedef std::shared_ptr<const Frame> ConstPtr;

		static const FrameMode MODE_UNDEFINED;
    		static const FrameMode MODE_GRAYSCALE;
    	   	static const FrameMode MODE_RGB;
   		static const FrameMode MODE_UYVY;
		static const FrameMode MODE_BGR;
		static const FrameMode MODE_RGB32;
		static const FrameMode RAW_MODES;
		static const FrameMode MODE_BAYER;
		static const FrameMode MODE_BAYER_RGGB;
		static const FrameMode MODE_BAYER_GRBG;
		static const FrameMode MODE_BAYER_BGGR;
		static const FrameMode MODE_BAYER_GBRG;
		static const FrameMode COMPRESSED_MODES;
		static const FrameMode MODE_PJPG;
		static const FrameMode MODE_JPEG;
		static const FrameMode MODE_PNG;

		static const FrameStatus STATUS_EMPTY;
		static const FrameStatus STATUS_VALID;
		static const FrameStatus STATUS_INVALID;


		Frame();
		~Frame();

		void SetFrameTime(T_Int64 time);
		T_Int64 GetFrameTime() const;

		void SetReceivedTime(T_Int64 time);
		T_Int64 GetReceivedTime() const;

		void SetDataDepth(T_UInt32 dataDepth);
		T_UInt32 GetDataDepth() const;

		void SetPixelSize( T_UInt32 pizelSize);
		T_UInt32 GetPixelSize() const;

		void SetRowSize(T_UInt32 rowSize);
		T_UInt32 GetRowSize() const;

		void SetFrameMode(FrameMode frameMode);
		FrameMode GetFrameMode() const;
		
		void SetFrameStatus(FrameStatus frameStatus);
		FrameStatus GetFrameStatus() const;

		void SetFrameSize(T_UInt16 width, T_UInt16 height);
		void SetFrameSize(FrameSize frameSize);
		T_UInt16 GetFrameWidth() const;
		T_UInt16 GetFrameHeight() const;
		FrameSize GetFrameSize() const;
		
		void AddAttribute(T_String data, T_String name);
		void ClearAttributes();
		void RemoveAttribute(int index);
		FrameAttribute GetAttribute(int index) const;
		unsigned GetNumberOfAttributes() const;
	
		void AddDataByte(byte data);
		void ClearData();
		byte GetDataByte(int index) const;
		int GetNumberOfDataBytes() const;
		
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
		static const int MAX_FRAME_ATTRIBUTES;
		static const int MAX_DATA_BYTE_SIZE;

		CTypes::Frame frame;

	};



}
#endif

/* Frame.hpp */
/** @} */
