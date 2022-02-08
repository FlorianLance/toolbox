
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
    Restart
};

struct Header{

    Header() = default;
    Header(std::int8_t* data){
        std::copy(data, data + sizeof(Header), reinterpret_cast<std::int8_t*>(this));
    }

    MessageType type;
    std::uint32_t totalSizeBytes = 0;
    std::uint16_t totalNumberPackets = 0;
    std::uint16_t currentPacketId = 0;
    std::uint16_t currentPacketSizeBytes = 0;
    std::uint32_t currentPacketTime = 0;
    std::uint32_t dataOffset = 0;
    std::int32_t idMessage = -1;

    static Header generate_mono_packet(MessageType type, size_t messageNbBytes);
};

struct UdpMessage{
    void copy_to_data(const Header &header, size_t messageNbBytes, std::vector<std::int8_t> &data) const;
    void read_from_data(std::int8_t *data, std::uint32_t messageNbBytes);
};

struct UdpInitFromManager : UdpMessage{

    UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket);
    UdpInitFromManager(std::int8_t *data, std::uint32_t messageNbBytes);

    std::array<char, 45> ipAdress;
    std::uint16_t port;
    std::uint16_t maxSizeUdpPacket;
};

struct UdpCommand : UdpMessage{

    UdpCommand() = default;
    UdpCommand(std::int8_t *data, std::uint32_t messageNbBytes);

    Command command;
};

struct UdpUpdateDeviceSettings : UdpMessage{

    UdpUpdateDeviceSettings() = default;
    UdpUpdateDeviceSettings(std::int8_t *data, std::uint32_t messageNbBytes);

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

struct UdpUpdateFiltersSettings : UdpMessage{
    UdpUpdateFiltersSettings() = default;
    UdpUpdateFiltersSettings(std::int8_t *data, std::uint32_t messageNbBytes);
    camera::K4::Filters filters;
};

struct UdpFeedbackMessage : UdpMessage{
    UdpFeedbackMessage() = default;
    UdpFeedbackMessage(std::int8_t *data, std::uint32_t messageNbBytes);
    MessageType receivedMessageType;
    Feedback feedback;
};

struct MultiPacketsMessageData{

    bool receivingFrame = false;
    size_t timeoutMs = 15;
    std::chrono::nanoseconds firstPacketTimestamp;
    size_t totalBytesReceived = 0;
    size_t nbPacketsReceived = 0;
    std::vector<std::int8_t> frameData;
    int idLastMultiPacketsMessageReceived = -1;

    bool process(const Header &header, std::int8_t *data, size_t nbBytes);
    bool all_received(const Header &header);

    std::shared_ptr<camera::K4::CompressedCloudFrame> generate_compressed_cloud_frame();
    std::shared_ptr<camera::K4::CompressedFullFrame> generate_compressed_full_frame();
};


}
