
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


#include "tcp_sender.hpp"

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

struct ScanerTcpSender::Impl{

    // tcp
    io_service ioService;
    std::unique_ptr<ip::tcp::socket> socket;
    boost::asio::ip::basic_resolver_results<ip::tcp> endpoint;
    std::string solvedAdress;
    bool connectionValid = false;
};

ScanerTcpSender::ScanerTcpSender(): i(std::make_unique<Impl>()){
}

ScanerTcpSender::~ScanerTcpSender(){
    clean_socket();
}


bool ScanerTcpSender::init_socket(std::string targetName, std::string writingPort){

    // reset socket if necessary
    if(i->connectionValid){
        clean_socket();
    }

    // init socket and service
    try{
        i->socket = std::make_unique<ip::tcp::socket>(i->ioService);
        i->socket->open(ip::tcp::v4());

        ip::tcp::resolver resolver(i->ioService);

        if(ip::host_name() == targetName){
            i->endpoint = resolver.resolve(ip::tcp::resolver::query(targetName = "localhost", writingPort, ip::tcp::resolver::canonical_name));
        }else{
            i->endpoint = resolver.resolve(ip::tcp::resolver::query(targetName, writingPort));
        }

    }catch (const boost::system::system_error& error){

        Logger::error(std::format("TcpSender: Cannot resolve target name {} with writing port {}, error message: {}.\n",
            targetName, writingPort, error.what()));
        clean_socket();
        return false;
    }

    // connect socket
    i->solvedAdress = i->endpoint->endpoint().address().to_string();
    try{
        boost::asio::connect(*i->socket.get(), i->endpoint);
    }catch (const boost::system::system_error& error){
        Logger::error(std::format("TcpSender: Cannot connect socket withg adress {}, error message: {}.\n",
            i->solvedAdress, error.what()));
        clean_socket();
        return false;
    }

    connection_state_signal(i->connectionValid = true);
    return true;
}

void ScanerTcpSender::clean_socket(){

    if(i->socket){
        try{
            i->ioService.stop();
            i->socket->shutdown(boost::asio::ip::tcp::socket::shutdown_send);
            std::this_thread::sleep_for (std::chrono::milliseconds(300));
            i->socket->close();
        }catch (const boost::system::system_error& error){
            Logger::error(std::format("TcpSender: Cannot shutdown socket with adress {}, error message: {}.\n",
                i->solvedAdress, error.what()));
        }
    }
    i->socket = nullptr;

    if(i->connectionValid){
        connection_state_signal(i->connectionValid = false);
    }
}

void ScanerTcpSender::send_data(std::int8_t *data, std::int32_t size){

    try{
        i->socket->send(boost::asio::buffer(&size, 4));
        i->socket->send(boost::asio::buffer(data, size));

    }catch (const boost::system::system_error& error){

        Logger::error(std::format("TcpSender: Cannot send data of size {} to adress {}, error message: {}.\n",
            size, i->solvedAdress, error.what()));
    }
}
