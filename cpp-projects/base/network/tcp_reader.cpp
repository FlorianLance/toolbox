

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

#include "tcp_reader.hpp"

// boost
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/system/system_error.hpp>
#include <boost/system/error_code.hpp>

// local
#include "utility/logger.hpp"

using namespace boost::asio;
using namespace boost::asio::detail;
using namespace boost::asio::ip;

using namespace tool;
using namespace tool::network;


struct TcpReader::Impl{

    // udp connection
    io_service ioService;
    std::unique_ptr<tcp::socket> socket = nullptr;
    tcp::endpoint endpoint;

    // reading thread
    std::unique_ptr<std::thread> thread = nullptr;
//    size_t currentBufferId = 0;
//    size_t buffersCount = 10000;
//    std::vector<std::vector<char>> buffers;

    // states
    std::atomic_bool connectionValid = false;
    std::atomic_bool isReading = false;
};

TcpReader::TcpReader() : i(std::make_unique<Impl>()){

}

TcpReader::~TcpReader(){

    if(is_reading()){
        stop_reading();
    }
    if(is_connected()){
        clean_socket();
    }
}

bool TcpReader::init_socket(std::string readingAdress, int readingPort){

    if(is_reading()){
        Logger::error(std::format("TcpReader: Cannot init socket while reading thread is still active.\n"));
        return false;
    }

    // reset socket if necessary
    if(i->connectionValid){
        clean_socket();
    }

    try {
        i->endpoint = tcp::endpoint(address::from_string(readingAdress), static_cast<unsigned short>(readingPort));

        // bind socket
        i->socket   = std::make_unique<tcp::socket>(i->ioService);
        i->socket->set_option(tcp::socket::reuse_address(true));
        i->socket->bind(i->endpoint);

    }catch (const boost::system::system_error &error){
        Logger::error(std::format("TcpReader: Cannot bind endpoint {}, {}, error message: {}.\n",
            readingAdress, readingPort, error.what()));
        clean_socket();
        return false;
    }

    connection_state_signal(i->connectionValid = true);
    return true;
}

void TcpReader::clean_socket(){

    if(i->socket){
        try {
            i->ioService.stop();
            i->socket->shutdown(udp::socket::shutdown_receive);
            std::this_thread::sleep_for (std::chrono::milliseconds(300));
            i->socket->close();
        }catch (const boost::system::system_error &error){
            Logger::error(std::format("TcpReader::clean_socket: Cannot shutdown socket, error message: {}.\n", error.what()));
        }
    }
    i->socket = nullptr;

    if(i->connectionValid){
        connection_state_signal(i->connectionValid = false);
    }
}

void TcpReader::start_reading(){

    if(is_reading()){
        Logger::error("TcpReader::start_reading: Reading already started.\n");
        return;
    }

    if(!is_connected()){
        Logger::error("TcpReader::start_reading: Socket not connected.\n");
        return;
    }

    if(i->thread == nullptr){
        i->thread = std::make_unique<std::thread>(&TcpReader::read_data, this);
    }else{
        Logger::error("TcpReader::start_reading: Cannot start reading, thread not cleaned.\n");
        return;
    }
}

void TcpReader::stop_reading(){

    if(!is_reading()){
        Logger::error("TcpReader::stop_reading: Reading not started.\n");
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

void TcpReader::process_packet(std::vector<char> *packet, size_t nbBytes){

}

bool TcpReader::is_reading() const noexcept{return i->isReading;}

bool TcpReader::is_connected() const noexcept{return i->connectionValid;}


void TcpReader::read_data(){

    i->isReading = true;

    std::vector<char> buffer(65535);
    while(i->isReading){

        if(!i->connectionValid){
            break;
        }

        size_t nbBytesReceived = 0;
        tcp::endpoint senderEndpoint;
        try {
            nbBytesReceived = read(*i->socket, boost::asio::buffer(buffer.data(),static_cast<size_t>(buffer.size())));

        } catch (const boost::system::system_error &error) {
            if(error.code() == boost::asio::error::timed_out){
                timeout_packet_signal();
            }else{
                Logger::error("TcpReader::read_data: Cannot read from socket, error message: {}\n", error.what());
                connection_state_signal(i->connectionValid = false);
            }
            continue;
        }

        if(nbBytesReceived == 0){
            Logger::warning("TcpReader::read_data: No bytes received.");
            continue;
        }

        //        senderEndpoint.address().to_v4().to_bytes();
        //        senderEndpoint.address().to_v6().to_bytes();
        //        senderEndpoint.protocol();
        //        endP.port();

        process_packet(&buffer, nbBytesReceived);
    }

    i->isReading = false;
}



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

//#include "tcp_reader_worker.hpp"

//// Qt
//#include <QTcpServer>
//#include <QTcpSocket>
//#include <QCoreApplication>
//#include <QDataStream>
//#include <QHostAddress>

//// qt-utility
//#include "qt_process.hpp"
//#include "qt_logger.hpp"

//using namespace tool;
//using namespace tool::network;


//struct ConnectionBuffer{
//    QByteArray buffer;
//    qint32 size = 0;
//};


//struct TcpReaderWorker::Impl{

//    QHash<QTcpSocket*, std::shared_ptr<ConnectionBuffer>> messages;
//    std::unique_ptr<QTcpServer> server = nullptr;
//    int readingPort;
//};

//TcpReaderWorker::TcpReaderWorker() : m_p(std::make_unique<Impl>()){
//}

//TcpReaderWorker::~TcpReaderWorker(){
//    disable_reading();
//}

//void TcpReaderWorker::clean_socket(){

//    wait_process(20);

//    if(i->server){
//        i->server->close();
//    }
//    i->server = nullptr;
//}

//bool TcpReaderWorker::enable_reading(int readingPort){

//    disable_reading();

//    i->readingPort = readingPort;
//    i->server = std::make_unique<QTcpServer>();
//    connect(i->server.get(), &QTcpServer::newConnection, this, &TcpReaderWorker::new_connection);

//    if(!i->server->listen(QHostAddress::Any, static_cast<quint16>(i->readingPort))){
//        clean_socket();
//        emit connected_state_signal("Any", i->readingPort, false);
//        return false;
//    }

//    emit connected_state_signal("Any", i->readingPort, true);
//    Logger::message("[TCP] Reading enabled");
//    return true;
//}


//void TcpReaderWorker::disable_reading(){

//    emit connected_state_signal("Any", i->readingPort, false);
//    clean_socket();
//    Logger::message("[TCP] Reading disabled");
//}

//void TcpReaderWorker::new_connection(){

//    while (i->server->hasPendingConnections()){

//        QTcpSocket *socket = i->server->nextPendingConnection();
//        connect(socket, &QTcpSocket::readyRead,    this, &TcpReaderWorker::ready_read);
//        connect(socket, &QTcpSocket::disconnected, this, &TcpReaderWorker::disconnected);
//        i->messages.insert(socket, std::make_shared<ConnectionBuffer>());
//        emit new_connection_signal(socket->localAddress().toString(), QString::number(socket->localPort()));
//    }
//}

//void TcpReaderWorker::disconnected(){

//    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
//    i->messages.remove(socket);
//    emit connection_ended_signal(socket->localAddress().toString(), QString::number(socket->localPort()));
//}

//qint32 array_to_int(QByteArray source){
//    std::reverse(source.begin(), source.end());
//    qint32 temp;
//    QDataStream data(&source, QIODevice::ReadWrite);
//    data >> temp;
//    return temp;
//}

//void TcpReaderWorker::ready_read(){

//    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());

//    if(!i->messages.contains(socket)){
//        Logger::error("Cannot read packet, socket not registered in hash.");
//        return;
//    }

//    ConnectionBuffer *message = i->messages.value(socket).get();
//    QByteArray *buffer = &message->buffer;
//    qint32 *s = &message->size;
//    qint32 size = *s;

//    while (socket->bytesAvailable() > 0){

//        buffer->append(socket->readAll());
//        while ((size == 0 && buffer->size() >= 4) || (size > 0 && buffer->size() >= size)){ //While can process data, process it

//            if (size == 0 && buffer->size() >= 4){ //if size of data has received completely, then store it
//                size = array_to_int(buffer->mid(0, 4));
//                *s = size;
//                buffer->remove(0, 4);
//            }
//            if (size > 0 && buffer->size() >= size){ // If data has received completely, then emit our SIGNAL with the data
//                QByteArray data = buffer->mid(0, size);
//                buffer->remove(0, size);
//                size = 0;
//                *s = size;

//                // process data
//                TcpPacket packet;
//                std::move(data.begin(), data.begin()+ sizeof(packet), reinterpret_cast<char*>(&packet));
//                emit tcp_packet_received_signal(packet);
//            }
//        }
//    }
//}


//#include "moc_tcp_reader_worker.cpp"


