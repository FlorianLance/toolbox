
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

enum MessageType : std::int8_t {
    manager_id = 0,
    update_device_settings,
    update_filters_settings,
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
    // cloud
    TMessageTypes
    {MessageType::manager_id,                   "manager_id"sv},
    {MessageType::update_device_settings,       "update_device_settings"sv},
    {MessageType::update_filters_settings,      "update_filters_settings"sv},
    {MessageType::compressed_cloud_frame_data,  "compressed_cloud_frame_data"sv},
    {MessageType::compressed_full_frame_data,   "compressed_full_frame_data"sv},
    {MessageType::command,                      "command"sv},
    {MessageType::feedback,                     "feedback"sv},
}};

[[maybe_unused]] static Name to_string(MessageType m) {
    return k4MessageTypes.at<0,1>(m);
}

enum K4Feedback : std::int8_t{
    valid = 0,
    timeout
};

enum K4Command : std::int8_t{
    Quit,
    Shutdown,
    Restart,
    Disconnect
};

struct K4UdpInitFromManager : UdpMonoPacketMessage{

    K4UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket, std::uint8_t idDevice);
    K4UdpInitFromManager(std::int8_t *data);

    std::array<char, 45> ipAdress;
    std::uint16_t port;
    std::uint16_t maxSizeUdpPacket;
    std::uint8_t idDevice;
};

struct K4UdpCommand : UdpMonoPacketMessage{

    K4UdpCommand(K4Command command) : command(command){}
    K4UdpCommand(std::int8_t *data);

    K4Command command;
};

struct K4UdpUpdateDeviceSettings : UdpMonoPacketMessage{

    K4UdpUpdateDeviceSettings() = default;
    K4UdpUpdateDeviceSettings(std::int8_t *data);

    // capture
    bool startDevice   = true;
    bool openCamera    = true;
    camera::K4Mode mode    = camera::K4Mode::Cloud_640x576;
    bool captureAudio  = true;
    bool captureIMU    = true;
    camera::K4CompressMode compressMode  = camera::K4CompressMode::Cloud;

    // network
    bool sendData      = true;

    // record
    bool record        = false;

    // display
    bool displayRGB    = false;
    bool displayDepth  = false;
    bool displayInfra  = false;
    bool displayCloud  = false;
};

struct K4UdpUpdateFiltersSettings : UdpMonoPacketMessage{
    K4UdpUpdateFiltersSettings() = default;
    K4UdpUpdateFiltersSettings(std::int8_t *data);
    camera::K4Filters filters;
};

struct K4UdpFeedbackMessage : UdpMonoPacketMessage{
    K4UdpFeedbackMessage() = default;
    K4UdpFeedbackMessage(std::int8_t *data);
    MessageType receivedMessageType;
    K4Feedback feedback;
};

struct K4UdpCompresedCloudFrameMessage : UdpMultiPacketsMessage{
    std::shared_ptr<camera::K4CompressedCloudFrame> generate_frame(std::int8_t *data);
    size_t initialize_data(std::shared_ptr<camera::K4CompressedCloudFrame> frame, std::vector<std::int8_t> &data);
};

struct K4UdpCompresedFullFrameMessage : UdpMultiPacketsMessage{
    std::shared_ptr<camera::K4CompressedFullFrame> generate_frame(std::int8_t *data);
    size_t initialize_data(std::shared_ptr<camera::K4CompressedFullFrame> frame, std::vector<std::int8_t> &data);
};

}
