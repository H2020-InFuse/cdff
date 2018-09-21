/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StringSequenceToStdVectorOfStringsConverter.cpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 *
 *  This is the class for type conversion from StringSequence to std::vector<std::string>.
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

#include "StringSequenceToStdVectorOfStringsConverter.hpp"
#include <Errors/Assert.hpp>

namespace Converters
{

const std::vector<std::string> StringSequenceToStdVectorOfStringsConverter::Convert(const asn1SccStringSequence& stringSequence)
{
    std::vector<std::string> strings_vector;
    for( int index = 0; index < stringSequence.nCount; index ++ )
    {
        asn1SccT_String string = stringSequence.arr[index];
        strings_vector.push_back(std::string (reinterpret_cast<char const*>(string.arr), string.nCount));
    }
    return strings_vector;
}

}

/** @} */