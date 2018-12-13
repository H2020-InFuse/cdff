/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

// TODO: Create a C++ wrapper for the transcompiled ASN.1 type StringSequence
// and use the wrapped type here instead of using the raw C type

/*!
 * @file StdVectorOfStringsToStringSequenceConverter.cpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 *
 *  This is the class for type conversion from std::vector<std::string> to StringSequence.
 *
 *
 * @{
 */

#include "StdVectorOfStringsToStringSequenceConverter.hpp"
#include <Errors/Assert.hpp>
#include <string.h>

namespace Converters
{

const asn1SccStringSequence StdVectorOfStringsToStringSequenceConverter::Convert(const std::vector<std::string>& stringVector)
{
    asn1SccStringSequence stringSequence;
    stringSequence.nCount = stringVector.size();
    unsigned int size = stringVector.size();
    for( unsigned int index = 0; index < size; index ++ )
    {
        std::string string = stringVector[index];
        stringSequence.arr[index].nCount = string.size();
        memcpy(stringSequence.arr[index].arr, string.data(), string.length());
    }
    return stringSequence;
}

}

/** @} */
