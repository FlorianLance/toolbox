
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
#include "kinect4_data.hpp"

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
static constexpr TupleArray<MessageType::SizeEnum, TMessageTypes> messageTypes = {{
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
    return messageTypes.at<0,1>(m);
}

enum Feedback : std::int8_t{
    valid = 0,
    timeout
};

enum Command : std::int8_t{
    Quit,
    Shutdown,
    Restart,
    Disconnect
};

struct UdpInitFromManager : UdpMonoPacketMessage{

    UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket, std::uint8_t idDevice);
    UdpInitFromManager(std::int8_t *data);

    std::array<char, 45> ipAdress;
    std::uint16_t port;
    std::uint16_t maxSizeUdpPacket;
    std::uint8_t idDevice;
};

struct UdpCommand : UdpMonoPacketMessage{

    UdpCommand(Command command) : command(command){}
    UdpCommand(std::int8_t *data);

    Command command;
};

struct UdpUpdateDeviceSettings : UdpMonoPacketMessage{

    UdpUpdateDeviceSettings() = default;
    UdpUpdateDeviceSettings(std::int8_t *data);

    // capture
    bool startDevice   = true;
    bool openCamera    = true;
    camera::K4::Mode mode    = camera::K4::Mode::Cloud_640x576;
    bool captureAudio  = true;
    bool captureIMU    = true;
    camera::K4::CompressMode compressMode  = camera::K4::CompressMode::Cloud;

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

struct UdpUpdateFiltersSettings : UdpMonoPacketMessage{
    UdpUpdateFiltersSettings() = default;
    UdpUpdateFiltersSettings(std::int8_t *data);
    camera::K4::Filters filters;
};

struct UdpFeedbackMessage : UdpMonoPacketMessage{
    UdpFeedbackMessage() = default;
    UdpFeedbackMessage(std::int8_t *data);
    MessageType receivedMessageType;
    Feedback feedback;
};


struct UdpCompresedCloudFrameMessage : UdpMultiPacketsMessage{
    std::shared_ptr<camera::K4::CompressedCloudFrame> generate_frame(std::int8_t *data);
    size_t initialize_data(std::shared_ptr<camera::K4::CompressedCloudFrame> frame, std::vector<std::int8_t> &data);
};

struct UdpCompresedFullFrameMessage : UdpMultiPacketsMessage{
    std::shared_ptr<camera::K4::CompressedFullFrame> generate_frame(std::int8_t *data);
    size_t initialize_data(std::shared_ptr<camera::K4::CompressedFullFrame> frame, std::vector<std::int8_t> &data);
};

}
