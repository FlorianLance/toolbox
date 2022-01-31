
#pragma once

// std
#include <memory>

// boost
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//using socket_type = boost::asio::ip::tcp::socket;

//class connection_handler {
//public:
//    void handle(std::shared_ptr<socket_type> socket, boost::system::error_code& ec);
//};

//class server {
//public:
//    explicit server(boost::asio::io_service& ios, const uint16_t port);
//    ~server();
//    void start();
//    void stop();
//private:
//    void run();
//    std::unique_ptr<std::thread> main_loop_thread_;

//    boost::asio::io_service& ios_;
//    boost::asio::ip::tcp::endpoint endpoint_;
//    boost::asio::ip::tcp::acceptor acceptor_;
//    connection_handler handler_;
//    std::atomic<bool> is_running{false};
//};



class TcpSession{

public:

    TcpSession(boost::asio::io_service& io_service) : socket_(io_service) {
    }

    boost::asio::ip::tcp::socket& socket();
    void start();
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
    void handle_write(const boost::system::error_code& error);

private:
    boost::asio::ip::tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};

class TcpServer{

public:
    TcpServer(boost::asio::io_service& io_service, short port);

    void handle_accept(TcpSession* new_session, const boost::system::error_code& error);

private:
    boost::asio::io_service& io_service_;
    boost::asio::ip::tcp::acceptor acceptor_;
};
