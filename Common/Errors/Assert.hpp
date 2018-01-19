/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Assert.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is a collection of ASSERT macros. Each macro will check whether a condition is true. If the condition is false a diagnostic message is added to the log, the logged data is visualized and  
 *  the program is halted.
 *   
 * @{
 */

#ifndef ASSERT_HPP
#define ASSERT_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Loggers/LoggerFactory.hpp>
#include <stdlib.h>
#include <sstream>
#include <exception>


/* --------------------------------------------------------------------------
 *
 * This Assert Exception is thrown when an assertion fail. It is not meant to
 * be caught. It is just used during testing, so that CATCH does not stop.
 *
 * --------------------------------------------------------------------------
 */
class AssertException: public std::exception
	{
 	virtual const char* what() const throw()
 	 	{
 	   	return "Assert Exception: A programmer-defined impossible condition has triggered";
 	 	}
	};


/* --------------------------------------------------------------------------
 *
 * Macros
 *
 * --------------------------------------------------------------------------
 */
#ifndef TESTING
	#define ABORT_PROGRAM() exit(EXIT_FAILURE);
#else
	#define ABORT_PROGRAM() throw AssertException();
#endif

#define ASSERT(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		LoggerFactory::GetLogger()->AddEntry(message); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}

#ifndef TESTING
	#define ASSERT_ON_TEST(condition, message) 
#else
	#define ASSERT_ON_TEST(condition, message) ASSERT(condition, message)
#endif

#define ASSERT_EQUAL(expression1, expression2, message) \
	{ \
	if ( (expression1) != (expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<message;\
		LoggerFactory::GetLogger()->AddEntry(stream.str()); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}	

#define WRITE_TO_LOG(message, value) \
	{ \
	std::stringstream stream; \
	stream << message << " " << (value); \
	LoggerFactory::GetLogger()->AddEntry(stream.str()); \
	}

#define PRINT_LOG() \
	{ \
	LoggerFactory::GetLogger()->Print(); \
	}

#define PRINT_TO_LOG(message, value) \
	{ \
	WRITE_TO_LOG(message, value) \
	PRINT_LOG() \
	LoggerFactory::GetLogger()->Clear(); \
	}

#define VERIFY(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		LoggerFactory::GetLogger()->AddEntry(message); \
		LoggerFactory::GetLogger()->Print(); \
		LoggerFactory::GetLogger()->Clear(); \
		} \
	}

/*
* VIEW_POINT_CLOUD is meant for debugging purposes. 
* When you use thid function, you should:
* 1) Include the following file: #include <pcl/visualization/pcl_visualizer.h>
* 2) Link the following libraries: pcl_visualization vtkRenderingCore-8.0 vtkCommonDataModel-8.0 vtkCommonMath-8.0 vtkCommonCore-8.0 boost_system
*/

#define VIEW_POINT_CLOUD(pointType, pointCloud) \
	{ \
	pcl::visualization::PCLVisualizer viewer ("test view"); \
    	pcl::visualization::PointCloudColorHandlerCustom< pointType > pclCloudColor(pointCloud, 255, 255, 255); \
    	viewer.addPointCloud(pointCloud, pclCloudColor, "input"); \
	\
    	while (!viewer.wasStopped ()) \
    		{ \
        	viewer.spinOnce(); \
        	pcl_sleep (0.01); \
    		} \
	}

#define VIEW_3_POINT_CLOUDS(pointType, pointCloud1, pointCloud2, pointCloud3) \
	{ \
	pcl::visualization::PCLVisualizer viewer ("test view"); \
    	pcl::visualization::PointCloudColorHandlerCustom< pointType > pclCloudColor1(pointCloud1, 255, 255, 255); \
    	pcl::visualization::PointCloudColorHandlerCustom< pointType > pclCloudColor2(pointCloud2, 255, 255, 0); \
    	pcl::visualization::PointCloudColorHandlerCustom< pointType > pclCloudColor3(pointCloud3, 255, 0, 0); \
    	viewer.addPointCloud(pointCloud1, pclCloudColor1, "input1"); \
    	viewer.addPointCloud(pointCloud2, pclCloudColor2, "input2"); \
    	viewer.addPointCloud(pointCloud3, pclCloudColor3, "input3"); \
	\
    	while (!viewer.wasStopped ()) \
    		{ \
        	viewer.spinOnce(); \
        	pcl_sleep (0.01); \
    		} \
	}

#endif

/* Assert.hpp */
/** @} */
