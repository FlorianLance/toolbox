
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
#include <memory>

// signals
#include "lsignal.h"

namespace tool::network{

class UdpSender {

public:

    UdpSender();
    ~UdpSender();

    // socket
    bool init_socket(std::string tagetName, std::string writingPort);//, size_t readingInterface);//, int readingPort);
    void clean_socket();

    // send
    size_t send_packet(std::int8_t *data, std::int32_t size);

private:

    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};
}

//#pragma once

//// Qt
//#include <QObject>

//// scaner
//#include "network/tcp_data.hpp"


//namespace tool::network{

//class UdpSenderWorker;
//using UdpSenderWorkerUP = std::unique_ptr<UdpSenderWorker>;

//class UdpSenderWorker : public QObject{
//    Q_OBJECT
//public:

//    UdpSenderWorker();
//    ~UdpSenderWorker();
//    void set_grabber_id(std::uint8_t id);

//public slots:

//    void enable_writing(QString writingAddress, int writingPort);
//    void disable_writing();
//    void send_frame(network::TcpPacket command, std::shared_ptr<camera::K2Frame> frame);

//signals:

//    void connected_state_signal(QString writingAddress, int writingPort, bool state);
//    void nb_bytes_sent_signal(qint64 timeStamp, size_t nbBytes);
//    void packets_failure_signal(size_t count);
//    void frame_sent_signal(std::int64_t time);

//private:

//    bool init_socket(QString address, quint16 port);
//    void clean_socket();

//private:

//    struct Impl;
//    std::unique_ptr<Impl> m_p = nullptr;
//};
//}
