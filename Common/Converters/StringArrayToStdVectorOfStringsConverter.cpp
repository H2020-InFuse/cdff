/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StringArrayToStdVectorOfStringsConverter.cpp
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


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */

#include "StringArrayToStdVectorOfStringsConverter.hpp"
#include <Errors/Assert.hpp>

namespace Converters
{

const std::vector<std::string> StringArrayToStdVectorOfStringsConverter::Convert(const BaseTypesWrapper::T_StringArray& stringArray)
{
    std::vector<std::string> strings_vector;
    for( int index = 0; index < stringArray.nCount; index ++ )
    {
        asn1SccT_String string = stringArray.arr[index];
        strings_vector.push_back(std::string (reinterpret_cast<char const*>(string.arr), string.nCount));
    }
    return strings_vector;
}

}

/** @} */