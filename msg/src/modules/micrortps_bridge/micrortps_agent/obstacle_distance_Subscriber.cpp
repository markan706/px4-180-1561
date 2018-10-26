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

/*!
 * @file obstacle_distance_Subscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "obstacle_distance_Subscriber.h"


obstacle_distance_Subscriber::obstacle_distance_Subscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

obstacle_distance_Subscriber::~obstacle_distance_Subscriber() {   Domain::removeParticipant(mp_participant);}

bool obstacle_distance_Subscriber::init()
{
    // Create RTPSParticipant

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0; //MUST BE THE SAME AS IN THE PUBLISHER
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("obstacle_distance_subscriber"); //You can put the name you want
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
            return false;

    //Register the type

    Domain::registerType(mp_participant, (TopicDataType*) &myType);

    // Create Subscriber

    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName(); //Must be registered before the creation of the subscriber
    Rparam.topic.topicName = "obstacle_distance_PubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, (SubscriberListener*) &m_listener);
    if(mp_subscriber == nullptr)
        return false;
    return true;
}

void obstacle_distance_Subscriber::SubListener::onSubscriptionMatched(Subscriber* sub, MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void obstacle_distance_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
        // Take data
        if(sub->takeNextData(&msg, &m_info))
        {
            if(m_info.sampleKind == ALIVE)
            {
                // Print your structure data here.
                ++n_msg;
                //std::cout << "Sample received, count=" << n_msg << std::endl;
                has_msg = true;

            }
        }
}

void obstacle_distance_Subscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

bool obstacle_distance_Subscriber::hasMsg()
{
    return m_listener.has_msg;
}

obstacle_distance_ obstacle_distance_Subscriber::getMsg()
{
    m_listener.has_msg = false;
    return m_listener.msg;
}
