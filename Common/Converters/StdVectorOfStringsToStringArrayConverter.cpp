/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StdVectorOfStringsToStringArrayConverter.cpp
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

#include "StdVectorOfStringsToStringArrayConverter.hpp"
#include <Errors/Assert.hpp>
#include <string.h>

namespace Converters
{

const BaseTypesWrapper::T_StringArray StdVectorOfStringsToStringArrayConverter::Convert(const std::vector<std::string> stringVector)
{
    BaseTypesWrapper::T_StringArray stringArray;
    stringArray.nCount = stringVector.size();
    unsigned int size = stringVector.size();
    for( unsigned int index = 0; index < size; index ++ )
    {
        std::string string = stringVector[index];
        stringArray.arr[index].nCount = string.size();
        memcpy(stringArray.arr[index].arr, string.data(), string.length());
    }
    return stringArray;
}

}

/** @} */