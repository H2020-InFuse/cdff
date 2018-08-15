/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECONSTRUCTION_INTERFACE_HPP
#define STEREORECONSTRUCTION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <Pointcloud.h>

#ifdef TESTING
	#include <opencv2/imgproc/imgproc.hpp>
#endif

namespace CDFF
{
namespace DFN
{
	/**
	 * DFN that turns a pair of stereo images into a reconstructed 3D scene
	 * (in the form of a 3D pointcloud)
	 */
	class StereoReconstructionInterface : public CDFF::DFN::DFNCommonInterface
	{
		public:

			StereoReconstructionInterface();
			virtual ~StereoReconstructionInterface();

			/**
			 * Send value to input port "left"
			 * @param left: left image captured by a stereo camera
			 */
			virtual void leftInput(const asn1SccFrame& data);
			/**
			 * Send value to input port "right"
			 * @param right: right image captured by a stereo camera
			 */
			virtual void rightInput(const asn1SccFrame& data);

			/**
			 * Query value from output port "pointcloud"
			 * @return pointcloud: reconstructed 3D pointcloud, in the
			 *         coordinate frame of the left camera
			 */
			virtual const asn1SccPointcloud& pointcloudOutput() const;

		protected:

			asn1SccFrame inLeft;
			asn1SccFrame inRight;
			asn1SccPointcloud outPointcloud;

		#ifdef TESTING

			protected:
				cv::Mat disparityMatrix;

			public:
				cv::Mat disparityMatrixOutput()
				{
					return disparityMatrix;
				}

		#endif // TESTING

	};
}
}

#endif // STEREORECONSTRUCTION_INTERFACE_HPP

/** @} */
