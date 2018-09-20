/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StdVectorOfStringsToStringArrayConverter.hpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from std::vector<std::string> to StringArray.
 *  
 *
 * @{
 */

#ifndef VECTOROFSTRINGS_TO_STRINGARRAY_CONVERTER_HPP
#define VECTOROFSTRINGS_TO_STRINGARRAY_CONVERTER_HPP


#include <BaseTypes.hpp>
#include <vector>

namespace Converters
{
    class StdVectorOfStringsToStringArrayConverter
	{
        public:
            virtual const BaseTypesWrapper::T_StringArray Convert(const std::vector<std::string> stringVector);
	};
}

#endif //VECTOROFSTRINGS_TO_STRINGARRAY_CONVERTER_HPP

/** @} */