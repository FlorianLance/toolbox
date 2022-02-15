
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

#include "udp_reader.hpp"

// boost
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/system/system_error.hpp>
#include <boost/system/error_code.hpp>

// local
#include "utility/logger.hpp"
#include "utility/format.hpp"

using namespace boost::asio;
using namespace boost::asio::detail;
using namespace boost::asio::ip;

using namespace tool;
using namespace tool::network;


struct UdpReader::Impl{

    // udp connection
    io_service ioService;
    std::unique_ptr<udp::socket> socket = nullptr;
    udp::endpoint endPoint;

    // reading thread
    std::unique_ptr<std::thread> thread = nullptr;
    size_t currentBufferId = 0;
    size_t buffersCount = 10000;
    std::vector<std::vector<char>> buffers;

    // states
    std::atomic_bool connectionValid = false;
    std::atomic_bool isReading = false;
};

UdpReader::UdpReader() : i(std::make_unique<Impl>()){

    // init buffers
    i->buffers.resize(i->buffersCount);
    for(auto &buffer : i->buffers){
        buffer.resize(9000);
    }
}

UdpReader::~UdpReader(){

    if(is_reading()){
        stop_reading();
    }
    if(is_connected()){
        clean_socket();
    }
}

bool UdpReader::init_socket(std::string readingAdress, int port){

    if(is_reading()){
        Logger::error(fmt("UdpReader: Cannot init socket while reading thread is still active.\n"));
        return false;
    }

    // reset socket if necessary
    if(i->connectionValid){
        clean_socket();
    }

    try {

        // open socket
        i->socket   = std::make_unique<udp::socket>(i->ioService);
        i->socket->open(udp::v4());
        i->socket->set_option(socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{500});
        i->socket->set_option(udp::socket::reuse_address(true));
        i->socket->set_option(udp::socket::receive_buffer_size(9000*50));//*10));

        // bind socket
        i->endPoint = udp::endpoint(address::from_string(readingAdress), static_cast<unsigned short>(port));
        i->socket->bind(i->endPoint);

    }catch (const boost::system::system_error &error){
        Logger::error(fmt("UdpReader: Cannot bind endpoint {}, {}, error message: {}.\n",
            readingAdress, port, error.what()));
        clean_socket();
        return false;
    }

    connection_state_signal(i->connectionValid = true);
    return true;
}

void UdpReader::clean_socket(){

    if(i->socket){
        try {
            i->ioService.stop();
            i->socket->shutdown(udp::socket::shutdown_receive);
            std::this_thread::sleep_for (std::chrono::milliseconds(300));
            i->socket->close();
        }catch (const boost::system::system_error &error){
            Logger::error(fmt("UdpReader::clean_socket: Cannot shutdown socket, error message: {}.\n", error.what()));
        }
    }
    i->socket = nullptr;

    if(i->connectionValid){
        connection_state_signal(i->connectionValid = false);
    }
}

void UdpReader::start_reading(){

    if(is_reading()){
        Logger::error("UdpReader::start_reading: Reading already started.\n");
        return;
    }

    if(!is_connected()){
        Logger::error("UdpReader::start_reading: Socket not connected.\n");
        return;
    }

    if(i->thread == nullptr){
        i->thread = std::make_unique<std::thread>(&UdpReader::read_data, this);
    }else{
        Logger::error("UdpReader::start_reading: Cannot start reading, thread not cleaned.\n");
        return;
    }
}

void UdpReader::stop_reading(){

    if(!is_reading()){
        Logger::error("UdpReader::stop_reading: Reading not started.\n");
        return;
    }

    if(i->thread != nullptr){
        if(i->thread->joinable()){
            i->isReading = false;
            i->thread->join();
        }
        i->thread = nullptr;
    }
}

void UdpReader::process_packet(std::vector<char> *packet, size_t nbBytes){
    static_cast<void>(packet);
    static_cast<void>(nbBytes);
}

bool UdpReader::is_reading() const noexcept{return i->isReading;}
bool UdpReader::is_connected() const noexcept{return i->connectionValid;}

void UdpReader::read_data(){

    i->isReading = true;

    while(i->isReading){

        if(!i->connectionValid){
            break;
        }

        auto buffer = &i->buffers[i->currentBufferId];
        size_t nbBytesReceived = 0;
        udp::endpoint senderEndpoint;
        try {
            // receive data
            nbBytesReceived = i->socket->receive_from(
                boost::asio::buffer(buffer->data(),static_cast<size_t>(buffer->size())),
                senderEndpoint
            );
            i->currentBufferId = (i->currentBufferId + 1) % i->buffersCount;

        } catch (const boost::system::system_error &error) {
            if(error.code() == boost::asio::error::timed_out){
                timeout_packet_signal();
            }else{
                Logger::error("UdpReader::read_data: Cannot read from socket, error message: {}\n", error.what());
                connection_state_signal(i->connectionValid = false);
            }
            continue;
        }

        if(nbBytesReceived == 0){
            Logger::warning("UdpReader::read_data: No bytes received.");
            continue;
        }

//        senderEndpoint.address().to_v4().to_bytes();
//        senderEndpoint.address().to_v6().to_bytes();
//        senderEndpoint.protocol();
//        endP.port();
//        Logger::message(fmt("receive from {} {}\n"),
//            senderEndpoint.address().to_v4().to_uint(),
//            senderEndpoint.port());
        process_packet(buffer, nbBytesReceived);
    }

    i->isReading = false;
}

