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
 * @file obstacle_distance_.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _OBSTACLE_DISTANCE__H_
#define _OBSTACLE_DISTANCE__H_

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
#if defined(obstacle_distance__SOURCE)
#define obstacle_distance__DllAPI __declspec( dllexport )
#else
#define obstacle_distance__DllAPI __declspec( dllimport )
#endif // obstacle_distance__SOURCE
#else
#define obstacle_distance__DllAPI
#endif
#else
#define obstacle_distance__DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure obstacle_distance_ defined by the user in the IDL file.
 * @ingroup OBSTACLE_DISTANCE_
 */
class obstacle_distance_
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport obstacle_distance_();
    
    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~obstacle_distance_();
    
    /*!
     * @brief Copy constructor.
     * @param x Reference to the object obstacle_distance_ that will be copied.
     */
    eProsima_user_DllExport obstacle_distance_(const obstacle_distance_ &x);
    
    /*!
     * @brief Move constructor.
     * @param x Reference to the object obstacle_distance_ that will be copied.
     */
    eProsima_user_DllExport obstacle_distance_(obstacle_distance_ &&x);
    
    /*!
     * @brief Copy assignment.
     * @param x Reference to the object obstacle_distance_ that will be copied.
     */
    eProsima_user_DllExport obstacle_distance_& operator=(const obstacle_distance_ &x);
    
    /*!
     * @brief Move assignment.
     * @param x Reference to the object obstacle_distance_ that will be copied.
     */
    eProsima_user_DllExport obstacle_distance_& operator=(obstacle_distance_ &&x);
    
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
     * @brief This function copies the value in member distances
     * @param _distances New value to be copied in member distances
     */
    inline eProsima_user_DllExport void distances(const std::array<uint16_t, 72> &_distances)
    {
        m_distances = _distances;
    }

    /*!
     * @brief This function moves the value in member distances
     * @param _distances New value to be moved in member distances
     */
    inline eProsima_user_DllExport void distances(std::array<uint16_t, 72> &&_distances)
    {
        m_distances = std::move(_distances);
    }

    /*!
     * @brief This function returns a constant reference to member distances
     * @return Constant reference to member distances
     */
    inline eProsima_user_DllExport const std::array<uint16_t, 72>& distances() const
    {
        return m_distances;
    }

    /*!
     * @brief This function returns a reference to member distances
     * @return Reference to member distances
     */
    inline eProsima_user_DllExport std::array<uint16_t, 72>& distances()
    {
        return m_distances;
    }
    /*!
     * @brief This function sets a value in member min_distance
     * @param _min_distance New value for member min_distance
     */
    inline eProsima_user_DllExport void min_distance(uint16_t _min_distance)
    {
        m_min_distance = _min_distance;
    }

    /*!
     * @brief This function returns the value of member min_distance
     * @return Value of member min_distance
     */
    inline eProsima_user_DllExport uint16_t min_distance() const
    {
        return m_min_distance;
    }

    /*!
     * @brief This function returns a reference to member min_distance
     * @return Reference to member min_distance
     */
    inline eProsima_user_DllExport uint16_t& min_distance()
    {
        return m_min_distance;
    }
    /*!
     * @brief This function sets a value in member max_distance
     * @param _max_distance New value for member max_distance
     */
    inline eProsima_user_DllExport void max_distance(uint16_t _max_distance)
    {
        m_max_distance = _max_distance;
    }

    /*!
     * @brief This function returns the value of member max_distance
     * @return Value of member max_distance
     */
    inline eProsima_user_DllExport uint16_t max_distance() const
    {
        return m_max_distance;
    }

    /*!
     * @brief This function returns a reference to member max_distance
     * @return Reference to member max_distance
     */
    inline eProsima_user_DllExport uint16_t& max_distance()
    {
        return m_max_distance;
    }
    /*!
     * @brief This function sets a value in member sensor_type
     * @param _sensor_type New value for member sensor_type
     */
    inline eProsima_user_DllExport void sensor_type(uint8_t _sensor_type)
    {
        m_sensor_type = _sensor_type;
    }

    /*!
     * @brief This function returns the value of member sensor_type
     * @return Value of member sensor_type
     */
    inline eProsima_user_DllExport uint8_t sensor_type() const
    {
        return m_sensor_type;
    }

    /*!
     * @brief This function returns a reference to member sensor_type
     * @return Reference to member sensor_type
     */
    inline eProsima_user_DllExport uint8_t& sensor_type()
    {
        return m_sensor_type;
    }
    /*!
     * @brief This function sets a value in member increment
     * @param _increment New value for member increment
     */
    inline eProsima_user_DllExport void increment(uint8_t _increment)
    {
        m_increment = _increment;
    }

    /*!
     * @brief This function returns the value of member increment
     * @return Value of member increment
     */
    inline eProsima_user_DllExport uint8_t increment() const
    {
        return m_increment;
    }

    /*!
     * @brief This function returns a reference to member increment
     * @return Reference to member increment
     */
    inline eProsima_user_DllExport uint8_t& increment()
    {
        return m_increment;
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
    eProsima_user_DllExport static size_t getCdrSerializedSize(const obstacle_distance_& data, size_t current_alignment = 0);


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
    std::array<uint16_t, 72> m_distances;
    uint16_t m_min_distance;
    uint16_t m_max_distance;
    uint8_t m_sensor_type;
    uint8_t m_increment;
};

#endif // _OBSTACLE_DISTANCE__H_