/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file XmlHelper.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is the XmlHelper class for utilities function during parsing of XML file with tinyxml2 library.
 *   
 * @{
 */

#ifndef XML_HELPER_HPP
#define XML_HELPER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Errors/Assert.hpp>
#include <tinyxml2.h>

using namespace tinyxml2;

namespace Common {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class XmlHelper
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
	
		/* --------------------------------------------------------------------------
		 * The following methods have the form:
		 *
		 * ExtractType(A, B, C, D): 
		 *
	 	 * They look for an XML element named B into the XML element A. Then it populates variable C with the value of B. 
		 * If the value is not of the right type, message D is written to log.
		 *
	 	 * --------------------------------------------------------------------------
		 */
		static void ExtractInt(XMLElement* xmlParent, const char* elementName, int& outputVariable, std::string errorMessage);
		static void ExtractFloat(XMLElement* xmlParent, const char* elementName, float& outputVariable, std::string errorMessage);
		static void ExtractDouble(XMLElement* xmlParent, const char* elementName, double& outputVariable, std::string errorMessage);
		static void ExtractBool(XMLElement* xmlParent, const char* elementName, bool& outputVariable, std::string errorMessage);
		static void ExtractString(XMLElement* xmlParent, const char* elementName, std::string& outputVariable, std::string errorMessage);

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
	
	};

}

#endif

/* XmlHelper.hpp */
/** @} */
