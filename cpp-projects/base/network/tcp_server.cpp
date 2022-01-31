

#include "tcp_server.hpp"

// std
#include <iostream>


//void connection_handler::handle(std::shared_ptr<socket_type> socket, boost::system::error_code& ec) {
//    std::cout << "start handling connection ..." << std::endl;

//    std::cout << "reading request ..." << std::endl;
//    boost::asio::streambuf buf;
//    boost::asio::read_until(*socket.get(), buf, "\n");

//    std::istream input{&buf};
//    std::string request;
//    std::getline(input, request);

//    std::cout << "request: " << request << std::endl;

//    std::cout << "sending response" << std::endl;
//    boost::asio::write(*socket.get(), boost::asio::buffer(request + "\n")); // echo

//    std::cout << "closing connection ..." << std::endl;
//    socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);

//    std::cout << "\n\n";
//}

//server::server(boost::asio::io_service& ios, const uint16_t port) :
//      ios_(ios),
//      endpoint_(boost::asio::ip::address_v4::any(), port),
//      acceptor_(ios_, endpoint_)
//{
//    std::cout << "server ctor: " << port << std::endl;
//}

//server::~server() {
//    std::cout << "server dtor" << std::endl;
//    stop();
//}

//void server::start() {

//    std::cout << "starting server ..." << std::endl;
//    main_loop_thread_.reset(
//        new std::thread([this]() {run();})
//    );
//}

//void server::run() {

//    std::cout << "start listening ..." << std::endl;
//    acceptor_.listen();

//    is_running.store(true);

//    while (is_running.load()) {

//        std::cout << "creating new socket ..." << std::endl;
//        auto socket = std::make_shared<socket_type>(ios_);

//        std::cout << "accepting connection ..." << std::endl;
//        acceptor_.accept(*socket.get());

//        // TODO create a thread
//        boost::system::error_code ec;
//        handler_.handle(socket, ec);
//    }

//    std::cout << "finish listenning" << std::endl;
//}

//void server::stop() {
//    std::cout << "stopping server ..." << std::endl;
//    is_running.store(false);
//    if (main_loop_thread_) {
//        main_loop_thread_->join();
//    }
//}

boost::asio::ip::tcp::socket &TcpSession::socket(){
    return socket_;
}

void TcpSession::start(){

    socket_.async_read_some(
        boost::asio::buffer(data_, max_length),
        boost::bind(&TcpSession::handle_read, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred
    ));
}

void TcpSession::handle_read(const boost::system::error_code &error, size_t bytes_transferred){

    if (!error){
        boost::asio::async_write(
            socket_,
            boost::asio::buffer(data_, bytes_transferred),
            boost::bind(&TcpSession::handle_write, this,
            boost::asio::placeholders::error)
        );
    } else{
        delete this;
    }
}

void TcpSession::handle_write(const boost::system::error_code &error){

    if (!error){
        socket_.async_read_some(
            boost::asio::buffer(data_, max_length),
            boost::bind(&TcpSession::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        ));
    }else{
        delete this;
    }
}

TcpServer::TcpServer(boost::asio::io_service &io_service, short port) :
    io_service_(io_service),
    acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)){

    TcpSession* new_session = new TcpSession(io_service_);
    acceptor_.async_accept(
        new_session->socket(),
        boost::bind(&TcpServer::handle_accept, this, new_session,
        boost::asio::placeholders::error
    ));
}

void TcpServer::handle_accept(TcpSession *new_session, const boost::system::error_code &error){

    if (!error){
        new_session->start();
        new_session = new TcpSession(io_service_);
        acceptor_.async_accept(
            new_session->socket(),
            boost::bind(&TcpServer::handle_accept, this, new_session,
            boost::asio::placeholders::error
        ));
    }else{
        delete new_session;
    }
}
