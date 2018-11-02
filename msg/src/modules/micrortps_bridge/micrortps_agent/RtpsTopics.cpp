/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RtpsTopics.h"

bool RtpsTopics::init()
{
    // Initialise subscribers
    if (_camera_trigger_sub.init()) {
        std::cout << "camera_trigger subscriber started" << std::endl;
    } else {
        std::cout << "ERROR starting camera_trigger subscriber" << std::endl;
        return false;
    }

    if (_obstacle_distance_sub.init()) {
        std::cout << "obstacle_distance subscriber started" << std::endl;
    } else {
        std::cout << "ERROR starting obstacle_distance subscriber" << std::endl;
        return false;
    }

    if (_optical_flow_sub.init()) {
        std::cout << "optical_flow subscriber started" << std::endl;
    } else {
        std::cout << "ERROR starting optical_flow subscriber" << std::endl;
        return false;
    }

    // Initialise publishers
    if (_rtps_send_distance_sensor_pub.init()) {
        std::cout << "rtps_send_distance_sensor publisher started" << std::endl;
    } else {
        std::cout << "ERROR starting rtps_send_distance_sensor publisher" << std::endl;
        return false;
    }

    return true;
}

void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
        case 137: // rtps_send_distance_sensor
        {
            rtps_send_distance_sensor_ st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _rtps_send_distance_sensor_pub.publish(&st);
        }
        break;
        default:
            printf("Unexpected topic ID to publish\n");
        break;
    }
}

bool RtpsTopics::hasMsg(uint8_t *topic_ID)
{
    if (nullptr == topic_ID) return false;

    *topic_ID = 0;
    while (_next_sub_idx < 3 && 0 == *topic_ID)
    {
        switch (_sub_topics[_next_sub_idx])
        {
            case 8: if (_camera_trigger_sub.hasMsg()) *topic_ID = 8; break;
            case 42: if (_obstacle_distance_sub.hasMsg()) *topic_ID = 42; break;
            case 44: if (_optical_flow_sub.hasMsg()) *topic_ID = 44; break;
            default:
                printf("Unexpected topic ID to check hasMsg\n");
            break;
        }
        _next_sub_idx++;
    }

    if (0 == *topic_ID)
    {
        _next_sub_idx = 0;
        return false;
    }

    return true;
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    bool ret = false;
    switch (topic_ID)
    {
        case 8: // camera_trigger
            if (_camera_trigger_sub.hasMsg())
            {
                camera_trigger_ msg = _camera_trigger_sub.getMsg();
                msg.serialize(scdr);
                ret = true;
            }
        break;
        case 42: // obstacle_distance
            if (_obstacle_distance_sub.hasMsg())
            {
                obstacle_distance_ msg = _obstacle_distance_sub.getMsg();
                msg.serialize(scdr);
                ret = true;
            }
        break;
        case 44: // optical_flow
            if (_optical_flow_sub.hasMsg())
            {
                optical_flow_ msg = _optical_flow_sub.getMsg();
                msg.serialize(scdr);
                ret = true;
            }
        break;
        default:
            printf("Unexpected topic ID '%hhu' to getMsg\n", topic_ID);
        break;
    }

    return ret;
}
