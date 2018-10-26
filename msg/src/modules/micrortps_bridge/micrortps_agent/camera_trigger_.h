// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file camera_trigger_.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _CAMERA_TRIGGER__H_
#define _CAMERA_TRIGGER__H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(camera_trigger__SOURCE)
#define camera_trigger__DllAPI __declspec( dllexport )
#else
#define camera_trigger__DllAPI __declspec( dllimport )
#endif // camera_trigger__SOURCE
#else
#define camera_trigger__DllAPI
#endif
#else
#define camera_trigger__DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure camera_trigger_ defined by the user in the IDL file.
 * @ingroup CAMERA_TRIGGER_
 */
class camera_trigger_
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport camera_trigger_();
    
    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~camera_trigger_();
    
    /*!
     * @brief Copy constructor.
     * @param x Reference to the object camera_trigger_ that will be copied.
     */
    eProsima_user_DllExport camera_trigger_(const camera_trigger_ &x);
    
    /*!
     * @brief Move constructor.
     * @param x Reference to the object camera_trigger_ that will be copied.
     */
    eProsima_user_DllExport camera_trigger_(camera_trigger_ &&x);
    
    /*!
     * @brief Copy assignment.
     * @param x Reference to the object camera_trigger_ that will be copied.
     */
    eProsima_user_DllExport camera_trigger_& operator=(const camera_trigger_ &x);
    
    /*!
     * @brief Move assignment.
     * @param x Reference to the object camera_trigger_ that will be copied.
     */
    eProsima_user_DllExport camera_trigger_& operator=(camera_trigger_ &&x);
    
    /*!
     * @brief This function sets a value in member timestamp
     * @param _timestamp New value for member timestamp
     */
    inline eProsima_user_DllExport void timestamp(uint64_t _timestamp)
    {
        m_timestamp = _timestamp;
    }

    /*!
     * @brief This function returns the value of member timestamp
     * @return Value of member timestamp
     */
    inline eProsima_user_DllExport uint64_t timestamp() const
    {
        return m_timestamp;
    }

    /*!
     * @brief This function returns a reference to member timestamp
     * @return Reference to member timestamp
     */
    inline eProsima_user_DllExport uint64_t& timestamp()
    {
        return m_timestamp;
    }
    /*!
     * @brief This function sets a value in member timestamp_utc
     * @param _timestamp_utc New value for member timestamp_utc
     */
    inline eProsima_user_DllExport void timestamp_utc(uint64_t _timestamp_utc)
    {
        m_timestamp_utc = _timestamp_utc;
    }

    /*!
     * @brief This function returns the value of member timestamp_utc
     * @return Value of member timestamp_utc
     */
    inline eProsima_user_DllExport uint64_t timestamp_utc() const
    {
        return m_timestamp_utc;
    }

    /*!
     * @brief This function returns a reference to member timestamp_utc
     * @return Reference to member timestamp_utc
     */
    inline eProsima_user_DllExport uint64_t& timestamp_utc()
    {
        return m_timestamp_utc;
    }
    /*!
     * @brief This function sets a value in member seq
     * @param _seq New value for member seq
     */
    inline eProsima_user_DllExport void seq(uint32_t _seq)
    {
        m_seq = _seq;
    }

    /*!
     * @brief This function returns the value of member seq
     * @return Value of member seq
     */
    inline eProsima_user_DllExport uint32_t seq() const
    {
        return m_seq;
    }

    /*!
     * @brief This function returns a reference to member seq
     * @return Reference to member seq
     */
    inline eProsima_user_DllExport uint32_t& seq()
    {
        return m_seq;
    }
    
    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const camera_trigger_& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;
    
private:
    uint64_t m_timestamp;
    uint64_t m_timestamp_utc;
    uint32_t m_seq;
};

#endif // _CAMERA_TRIGGER__H_