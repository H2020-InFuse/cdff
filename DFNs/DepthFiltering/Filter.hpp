/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DEPTHFILTERING_FILTER_HPP
#define DEPTHFILTERING_FILTER_HPP

#include "DepthFilteringInterface.hpp"
#include <Frame.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace CDFF
{
namespace DFN
{
namespace DepthFiltering
{
	/**
	 * Filtering of a 2D Depth image
	 *
	 * @param generalParameters.kernelSize
	 */

	class Filter : public DepthFilteringInterface
	{
		public:

			Filter();

			virtual void configure();
			virtual void process();

		private:

			struct FilterOptionsSet
			{
				int kernelSize;
			};

			Helpers::ParametersListHelper parametersHelper;
			FilterOptionsSet parameters;
            static const FilterOptionsSet DEFAULT_PARAMETERS;


            Converters::FrameToMatConverter frameToMat;
            Converters::MatToFrameConverter matToFrame;

			cv::Mat ApplyFilter(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputImage);
    };
}
}
}

#endif // DEPTHFILTERING_FILTER_HPP

/** @} */
