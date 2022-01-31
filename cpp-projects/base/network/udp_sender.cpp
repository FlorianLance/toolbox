
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


#include "udp_sender.hpp"

// std
#include <format>

// boost
#include <boost/asio.hpp>

// local
#include "utility/logger.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;

using namespace boost::asio;

using namespace tool;
using namespace tool::network;

struct UdpSender::Impl{
    io_service ioService;
    std::unique_ptr<ip::udp::socket> socket;
    boost::asio::ip::basic_resolver_results<ip::udp> endpoint;
    std::string solvedAdress;
};

UdpSender::UdpSender(): i(std::make_unique<Impl>()){
}

UdpSender::~UdpSender(){
    clean_socket();
}


bool UdpSender::init_socket(std::string targetName, std::string writingPort){

    // reset socket if necessary
    if(i->socket){
        clean_socket();
    }

    // init socket and service
    try{
        i->socket = std::make_unique<ip::udp::socket>(i->ioService);
        i->socket->open(ip::udp::v4());
        i->socket->set_option(detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 5 });
        i->socket->set_option(ip::udp::socket::reuse_address(true));
        i->socket->set_option(ip::udp::socket::send_buffer_size(9000*1000));
        i->socket->set_option(ip::udp::socket::receive_buffer_size(9000*1000));

        ip::udp::resolver resolver(i->ioService);

        if(ip::host_name() == targetName){
            i->endpoint = resolver.resolve(ip::udp::resolver::query(
                targetName = "localhost", writingPort, ip::udp::resolver::canonical_name));
        }else{
            i->endpoint = resolver.resolve(
                ip::udp::resolver::query(targetName, writingPort));
        }

    }catch (const boost::system::system_error& error){

        Logger::error(std::format("UdpSender: Cannot resolve target name {} with writing port {}, error message: {}.\n",
            targetName, writingPort, error.what()));
        clean_socket();
        return false;
    }

    return true;
}

void UdpSender::clean_socket(){

    if(i->socket){
        try{
            i->ioService.stop();
            i->socket->shutdown(boost::asio::ip::udp::socket::shutdown_send);
            std::this_thread::sleep_for (std::chrono::milliseconds(300));
            i->socket->close();
        }catch (const boost::system::system_error& error){
            Logger::error(std::format("UdpSender: Cannot shutdown socket with adress {}, error message: {}.\n",
                i->solvedAdress, error.what()));
        }
    }
    i->socket = nullptr;
}

size_t UdpSender::send_packet(int8_t *data, int32_t size){

    size_t bytesNbSent=0;
    try{
        bytesNbSent = i->socket->send_to(boost::asio::buffer(data, size), i->endpoint->endpoint());
    } catch (const boost::system::system_error& error) {
        Logger::error(std::format("UdpSender::send_data: Cannot sent data to endpoint {}, error message: {}.\n",
            i->endpoint->endpoint().address().to_string(),
            error.what())
        );
    }
    return bytesNbSent;
}
