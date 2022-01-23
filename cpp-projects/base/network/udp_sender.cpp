
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

    // tcp
    io_service ioService;
    std::unique_ptr<ip::udp::socket> socket;
    boost::asio::ip::basic_resolver_results<ip::udp> endpoint;
    std::string solvedAdress;
    bool connectionValid = false;
};

UdpSender::UdpSender(): i(std::make_unique<Impl>()){
}

UdpSender::~UdpSender(){
    clean_socket();
}


bool UdpSender::init_socket(std::string targetName, std::string writingPort){

    // reset socket if necessary
    if(i->connectionValid){
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

    connection_state_signal(i->connectionValid = true);
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

    if(i->connectionValid){
        connection_state_signal(i->connectionValid = false);
    }
}
//#include "udp_sender_worker.hpp"

//// std
//#include <execution>
//#include <iostream>
//#include <chrono>

//// boost
//#include <boost/asio.hpp>
//#include <boost/array.hpp>
//#include <boost/bind.hpp>

//// Qt
//#include <QCoreApplication>
//#include <QElapsedTimer>
//#include <QDateTime>

//// qt-utility
//#include "qt_logger.hpp"

//// local
//#include "network/udp_data.hpp"

//using namespace std::chrono;
//using namespace boost::asio::ip;

//using namespace tool::network;
//using namespace tool::geo;
//using namespace tool::camera;

//struct UdpSenderWorker::Impl{

//    std::uint8_t grabberId = 0;
////    std::mutex locker;

//    // io
//    boost::asio::io_service ioService;
//    std::unique_ptr<udp::socket> socket = nullptr;
//    udp::endpoint endpoint;
//    QString writingAddress;
//    int writingPort;
//    std::atomic_bool connectionValid = false;

//    // camera
//    std::shared_ptr<K2Frame> lastFrame = nullptr;

//    // buffer data to send
//    uint32_t frameId = 0;
//    std_v1<unsigned char> bufferData;
//    std_v1<QByteArray> packets;
//    UdpHeader header;
//};

//UdpSenderWorker::UdpSenderWorker() : m_p(std::make_unique<Impl>()){
//}

//UdpSenderWorker::~UdpSenderWorker(){
//    disable_writing();
//}

//void UdpSenderWorker::set_grabber_id(uint8_t id){
//    i->grabberId = id;
//}

//void UdpSenderWorker::enable_writing(QString writingAddress, int writingPort){

////    std::unique_lock<std::mutex> lock(i->locker);
//    i->writingAddress = writingAddress;
//    i->writingPort    = writingPort;

//    clean_socket();
//    if(!init_socket(writingAddress, static_cast<quint16>(writingPort))){
//        clean_socket();
//        emit connected_state_signal(i->writingAddress, i->writingPort,  i->connectionValid = false);
//    }else{
//        emit connected_state_signal(i->writingAddress, i->writingPort, i->connectionValid = true);
//    }
//}

//void UdpSenderWorker::disable_writing(){

////    std::unique_lock<std::mutex> lock(i->locker);
//    clean_socket();
//    emit connected_state_signal(i->writingAddress, i->writingPort, i->connectionValid = false);
//}

//bool UdpSenderWorker::init_socket(QString address, quint16 port){

//    try{
//        // init socket
//        i->socket   = std::make_unique<udp::socket>(i->ioService);

//        // init endpoint
//        i->endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(address.toStdString()), port);

//        // open socket
//        i->socket->open(udp::v4());
//        i->socket->set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 5 });
//        i->socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
//        i->socket->set_option(udp::socket::send_buffer_size(9000*1000));
//        i->socket->set_option(udp::socket::receive_buffer_size(9000*1000));


//    }catch (const boost::system::system_error& error){
//        Logger::error(QSL("[UDP] init_socket: ") % QString(error.what()));
//        return false;
//    }

//    return true;
//}

//void UdpSenderWorker::clean_socket(){

//    if(i->socket){
//        try{
//            i->ioService.stop();
//            if(i->socket->is_open()){
//                i->socket->close();
//            }
//        }catch (const boost::system::system_error& error){
//            Logger::error(QSL("[UDP] clean_socket: ") % QString(error.what()));
//        }
//    }
//    i->socket = nullptr;
//}


//void UdpSenderWorker::send_frame(TcpPacket command, std::shared_ptr<K2Frame> frame){

//    if(!i->socket || !i->connectionValid){
//        Logger::error("Can't send frame, socket valid.");
//        return;
//    }

//    // get last frame
//    i->lastFrame = frame;

//    if(i->lastFrame == nullptr){ // no frame to send
//        return;
//    }

//    // start timer
//    auto timeStart = high_resolution_clock::now();

//    // check mode
//    if(i->lastFrame->mode != command.frameMode){
//        Logger::error("TCP packet mode and last frame mode different.");
//        return;
//    }

//    // store camera id
//    i->header.idGrabber = i->grabberId;

//    // set intrinsics
//    i->header.intrinsics1 = i->lastFrame->intrinsics[0]; // focalLengthX
//    i->header.intrinsics2 = i->lastFrame->intrinsics[1]; // focalLengthY
//    i->header.intrinsics3 = i->lastFrame->intrinsics[2]; // principalPointX
//    i->header.intrinsics4 = i->lastFrame->intrinsics[3]; // principalPointY

//    // data to send
//    char *data = nullptr;

//    const auto mode = i->lastFrame->mode;
//    if(mode == K2FrameRequest::compressed_color_1920x1080 || mode == K2FrameRequest::compressed_color_512x424){

//        data = reinterpret_cast<char*>(i->lastFrame->compressedImage.data());
//        i->header.sizeFullData = i->lastFrame->jpegColorSize;

//    }else if(mode == K2FrameRequest::depth_512x424){

//        data = reinterpret_cast<char*>(i->lastFrame->depthData->data());
//        i->header.sizeFullData = i->lastFrame->depthData->size()*sizeof(std::uint16_t);

//    }else if(mode == K2FrameRequest::infra_512x424 || mode == K2FrameRequest::long_exposure_infra_512x424){

//        data = reinterpret_cast<char*>(i->lastFrame->infraData->data());
//        i->header.sizeFullData = i->lastFrame->infraData->size()*sizeof(std::uint16_t);

//    }else if(mode == K2FrameRequest::compressed_color_cloud || mode == K2FrameRequest::compressed_color_mesh){

//        // store compressed data sizes
//        i->header.size1 = i->lastFrame->compressedDepthSize;
//        i->header.size2 = i->lastFrame->jpegColorSize;

//        const size_t sizeDepth16    = i->lastFrame->compressedDepthSize*2;
//        const size_t sizeDepth8     = sizeDepth16*2;
//        const size_t sizeColor8     = i->lastFrame->jpegColorSize;

//        const size_t bufferSize8 = sizeDepth8 + sizeColor8 + k2_bodies_joints_data_size8;
//        if(i->bufferData.size() < bufferSize8){
//            i->bufferData.resize(bufferSize8);
//        }

//        // depth
//        std::copy(
//            i->lastFrame->compressedDepthData.data(),
//            i->lastFrame->compressedDepthData.data() + sizeDepth16,
//            reinterpret_cast<std::uint16_t*>(i->bufferData.data())
//        );

//        // color
//        std::copy(
//            i->lastFrame->compressedImage.data(),
//            i->lastFrame->compressedImage.data() + i->lastFrame->jpegColorSize,
//            i->bufferData.data() + sizeDepth8
//        );

//        // joints
//        size_t id = 0;
//        Joint4x64 *bodyP = reinterpret_cast<Joint4x64*>(&i->bufferData[sizeDepth8 + sizeColor8]);
//        for(const auto &body : i->lastFrame->bodiesData){
//            for(const auto &joint : body.joints){
//                bodyP[id++] = joint_to_int(body, joint.first, joint.second);
//            }
//        }

//        i->header.sizeFullData = bufferSize8 * sizeof (std::uint8_t);
//        data = reinterpret_cast<char*>(i->bufferData.data());

//    }else{
//        Logger::error("Camera mode not managed.");
//        return;
//    }


//    // define packet sizes
//    const int sizePacket       = command.sizeUdpPackets;
//    const int sizePacketHeader = sizeof (UdpHeader);
//    const int sizePacketData   = sizePacket - sizePacketHeader;

//    unsigned short nbPacketsNeeded = static_cast<unsigned short>(i->header.sizeFullData/to_unsigned(sizePacketData));
//    const unsigned short rest      = static_cast<unsigned short>(i->header.sizeFullData%to_unsigned(sizePacketData));
//    if(rest > 0){
//        ++nbPacketsNeeded;
//    }

//    // resize packets array
//    if(i->packets.size() < nbPacketsNeeded){
//       i->packets.resize(nbPacketsNeeded);
//       for(auto &p : i->packets){
//           if(p.size() != sizePacket){
//               p.resize(sizePacket);
//           }
//       }
//    }

//    // init header
//    i->header.frameId        = i->frameId;
//    i->header.frameMode      = command.frameMode;
//    i->header.offset         = 0;
//    i->header.totalNbPackets = nbPacketsNeeded;

//    // fill packets with data
//    for(size_t idP = 0; idP < i->header.totalNbPackets; ++idP){

//        // set packet timestamp relative to kinect time
//        i->header.timeStamp = (std::chrono::high_resolution_clock::now().time_since_epoch().count()-i->lastFrame->timeStampGetFrame);

//        // get current data paquet size and id
//        i->header.sizePacketData = to_unsigned(sizePacketData);
//        if(idP == nbPacketsNeeded-1){ // last case
//            if(rest > 0){
//                i->header.sizePacketData = rest;
//            }
//        }
//        i->header.idPacket = static_cast<uint16_t>(idP);

//        // write header into packet
//        std::copy(reinterpret_cast<char*>(&i->header), reinterpret_cast<char*>(&i->header) + sizeof(UdpHeader), i->packets[idP].begin());

//        // write data into packet
////        std::copy(data + i->header.offset, data + i->header.offset + i->header.sizePacketData, i->packets[idP].begin() + sizeof(UdpHeader));
//        std::move(data + i->header.offset, data + i->header.offset + i->header.sizePacketData, i->packets[idP].begin() + sizeof(UdpHeader));
////        std::move(data + header.offset, data + header.offset + header.sizePacketData, i->packets[idP].begin() + sizeof(UdpHeader));

//        // increment data offset
//        i->header.offset += i->header.sizePacketData;
//    }

//    // send packets
//    size_t countFailure = 0;
//    for(size_t idP = 0; idP < i->header.totalNbPackets; ++idP){
//        size_t bytesNbSent=0;
//        try{
//            bytesNbSent = i->socket->send_to(boost::asio::buffer(i->packets[idP].data(), static_cast<size_t>(sizePacket)), i->endpoint);
//        } catch (const boost::system::system_error& error) {
//            Logger::error(QStringLiteral("[UDP] send_frame: ") + QString(error.what()));
//            ++countFailure;
//        }
//        emit nb_bytes_sent_signal(QDateTime::currentMSecsSinceEpoch(), bytesNbSent);
//    }

//    // increment frame counter
//    ++i->frameId;

//    emit packets_failure_signal(countFailure);
//    emit frame_sent_signal(duration_cast<microseconds>(high_resolution_clock::now() - timeStart).count());
//}


//#include "moc_udp_sender_worker.cpp"
