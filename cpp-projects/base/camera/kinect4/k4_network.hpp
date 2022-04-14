
/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// std
#include <string>
#include <chrono>

// local
#include "network/udp_header.hpp"
#include "k4_frame.hpp"

namespace tool::network {

using namespace std::literals::string_view_literals;

enum FeedbackType : std::int8_t{
    message_received,
    timeout,
    disconnect,
    quit,
    shutdown,
    restart
};

enum MessageType : std::int8_t {
    init_network_infos = 0,
    update_config_settings,
    update_device_settings,
    update_actions_settings,
    update_filters,
    compressed_cloud_frame_data,
    compressed_full_frame_data,
    command,
    feedback,    
    SizeEnum
};

using Name = std::string_view;
using TMessageTypes = std::tuple<
    MessageType,                                Name>;
static constexpr TupleArray<MessageType::SizeEnum, TMessageTypes> k4MessageTypes = {{
    TMessageTypes
    {MessageType::init_network_infos,           "init_network_infos"sv},
    {MessageType::update_config_settings,       "update_config_settings"sv},
    {MessageType::update_device_settings,       "update_device_settings"sv},
    {MessageType::update_actions_settings,       "update_action_settings"sv},
    {MessageType::update_filters,               "update_filters"sv},
    {MessageType::compressed_cloud_frame_data,  "compressed_cloud_frame_data"sv},
    {MessageType::compressed_full_frame_data,   "compressed_full_frame_data"sv},
    {MessageType::command,                      "command"sv},
    {MessageType::feedback,                     "feedback"sv},
}};

[[maybe_unused]] static Name to_string(MessageType m) {
    return k4MessageTypes.at<0,1>(m);
}

enum K4Command : std::int8_t{
    Quit,
    Shutdown,
    Restart,
    Disconnect
};

struct K4NetworkInfos{
    K4NetworkInfos() = default;
    K4NetworkInfos(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket) : port(port), maxSizeUdpPacket(maxSizeUdpPacket) {
        std::fill(ipAdress.begin(), ipAdress.end(), ' ');
        if(ipAdressStr.size() <= 45){
            std::copy(std::begin(ipAdressStr), std::end(ipAdressStr), ipAdress.begin());
        }
    }
    std::array<char, 45> ipAdress;
    std::uint16_t port;
    std::uint16_t maxSizeUdpPacket;
};

struct K4Feedback{
    MessageType receivedMessageType;
    FeedbackType feedback;
};

template <typename T>
struct K4UdpMessage : UdpMonoPacketMessage{
    K4UdpMessage(T d) : data(d){}
    K4UdpMessage(std::int8_t *data){init_packet_from_data(data, sizeof(K4UdpMessage<T>));}
    T data;
};

using K4UdpNetworkInfos    = K4UdpMessage<K4NetworkInfos>;
using K4UdpCommand         = K4UdpMessage<K4Command>;
using K4UdpFeedback        = K4UdpMessage<K4Feedback>;

using K4UdpConfigSettings  = K4UdpMessage<camera::K4ConfigSettings>;
using K4UdpDeviceSettings  = K4UdpMessage<camera::K4DeviceSettings>;
using K4UdpActionsSettings  = K4UdpMessage<camera::K4ActionsSettings>;
using K4UdpFilters         = K4UdpMessage<camera::K4Filters>;


//struct K4UdpCompresedCloudFrameMessage : UdpMultiPacketsMessage{
//    size_t initialize_data(camera::K4CompressedCloudFrame *frame, std::vector<std::int8_t> &data){
//        frame->initialize_data(data);
//    }
//};

//struct K4UdpCompresedCloudFrameMessage : UdpMultiPacketsMessage{

//    std::unique_ptr<camera::K4CompressedCloudFrame> generate_frame(std::int8_t *data);
//};

//struct K4UdpCompresedFullFrameMessage : UdpMultiPacketsMessage{
//    size_t initialize_data(camera::K4CompressedFullFrame *frame, std::vector<std::int8_t> &data);
//    std::unique_ptr<camera::K4CompressedFullFrame> generate_frame(std::int8_t *data);
//};

}
