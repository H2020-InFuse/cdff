/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ParametersInterface.hpp
 * @date 24/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class creates a parameters choice interface by using basic GUI functions from OpenCV library. 
 * This class creates a sliders for each parameter, the sliders are grouped into distinct windows according to the parameter group. (Note that, a slider is the only GUI input method provided by OpenCV.)
 * This class offers methods for printing parameters to an XML file (which is what a DFN requires).
 * The constructor requires a name for the interface, this will also be the name of the root element of the XML file.
 * The parameters need to belong to a group. A parameter can be one of:
 * 1) int: this is the basic type, the values can go from 0 to a user specified maximum;
 * 2) bool: a boolean value is defines as an int from 0 to 1. It will have the same GUI interface, but different XML output;
 * 3) real or double: the user needs to specify a resolution. A resolution greater than 10^-7 yields a double, the other resolutions yield a real.
 *
 * @{
 */

#ifndef PARAMTERS_INTERFACE_HPP
#define PARAMTERS_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class ParametersInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:	
		ParametersInterface() = delete;
		explicit ParametersInterface(const std::string& applicationName);
		~ParametersInterface();
		void AddParameter(const std::string& groupName, const std::string& name, int defaultValue, int maxValue);
		void AddParameter(const std::string& groupName, const std::string& name, double defaultValue, double maxValue, double resolution);
		void AddSignedParameter(const std::string& groupName, const std::string& name, double defaultValue, double maxValue, double resolution);
        void AddSignedParameter(const std::string& groupName, const std::string& name, double defaultValue, double maxValue, double minValue, double resolution);
		void CreateTrackbars();
		void SaveToYaml(const std::string& filePath);

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
	
		enum ParameterType
			{
			BOOL_TYPE,
			INT_TYPE,
			FLOAT_TYPE,
			DOUBLE_TYPE
			};

		struct Parameter
			{
			std::string name;
			int value;
			ParameterType type;
			int maxValue;
			int displacement;
			double resolution;
			};

		std::map<std::string, std::vector<Parameter> > parameterGroupsMap;
		std::string applicationName;

		void AddParameter(const std::string& groupName, Parameter parameter);
	};

#endif

/* ParametersInterface.hpp */
/** @} */
