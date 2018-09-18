/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StringArrayToStdVectorOfStringsConverter.hpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from StringArray to std::vector<std::string>.
 *  
 *
 * @{
 */

#ifndef STRINGARRAY_TO_VECTOROFSTRINGS_CONVERTER_HPP
#define STRINGARRAY_TO_VECTOROFSTRINGS_CONVERTER_HPP


#include <BaseTypes.hpp>
#include <vector>

namespace Converters
{
    class StringArrayToStdVectorOfStringsConverter
	{
        public:
            virtual const std::vector<std::string> Convert(const BaseTypesWrapper::T_StringArray& stringArray);
	};
}

#endif //STRINGARRAY_TO_VECTOROFSTRINGS_CONVERTER_HPP

/** @} */