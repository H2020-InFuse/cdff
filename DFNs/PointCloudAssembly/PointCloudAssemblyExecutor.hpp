/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_EXECUTOR_HPP
#define POINTCLOUDASSEMBLY_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "PointCloudAssemblyInterface.hpp"
#include <PointCloud.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the assembly of point cloud. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFirstCloud: first input point cloud;
* @param inputSecondCloud: second input point cloud;
* @param outputAssembledCloud: output assembled point cloud.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class PointCloudAssemblyExecutor
    {
        public:

            PointCloudAssemblyExecutor(PointCloudAssemblyInterface* dfn);
            ~PointCloudAssemblyExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputFirstCloud, PointCloudWrapper::PointCloudConstPtr inputSecondCloud, PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
	    void Execute(PointCloudWrapper::PointCloudConstPtr inputFirstCloud, PointCloudWrapper::PointCloudConstPtr inputSecondCloud, PointCloudWrapper::PointCloudPtr outputAssembledCloud);
	    void Execute(const PointCloudWrapper::PointCloud& inputFirstCloud, const PointCloudWrapper::PointCloud& inputSecondCloud, PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
	    void Execute(const PointCloudWrapper::PointCloud& inputFirstCloud, const PointCloudWrapper::PointCloud& inputSecondCloud, PointCloudWrapper::PointCloud& outputAssembledCloud);

        private:

            PointCloudAssemblyInterface* dfn;
    };
}
}

#endif // POINTCLOUDASSEMBLY_EXECUTOR_HPP

/** @} */
