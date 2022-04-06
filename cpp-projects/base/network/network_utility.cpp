


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

#include "network_utility.hpp"

// boost
#include <boost/asio.hpp>
#include <boost/asio/ip/host_name.hpp>

// local
#include "utility/logger.hpp"
#include "utility/format.hpp"

using namespace boost::asio;
using namespace tool::network;

//Endpoint UDP::query(std::string targetName, std::string port){

//    io_service ioService;
//    ip::udp::resolver resolver(ioService);

//    ip::basic_resolver_results<ip::udp> endPointRes;
//    try{
//        if(ip::host_name() == targetName){
//            endPointRes = resolver.resolve(
//                ip::udp::resolver::query(
//                    targetName = "localhost",
//                    port,
//                    ip::udp::resolver::canonical_name
//                )
//            );
//        }else{
//            endPointRes = resolver.resolve(
//                    ip::udp::resolver::query(
//                    targetName,
//                    port
//                )
//            );
//        }
//    }catch (const boost::system::system_error &error){
//        Logger::error(std::format("UDP::query: Cannot solve target name {}, error message: {}\n", targetName, error.what()));
//    }

//    ioService.stop();

//    auto endPoint = endPointRes->endpoint();
//    return {
//        endPoint.address().is_v6() ? Protocol::ipv6 : (endPoint.address().is_v4() ? Protocol::ipv4 : Protocol::unknow),
//        endPoint.address().to_string(),
//        endPoint.port()
//    };
//}



std::vector<Interface> Interface::list_local_interfaces(Protocol protocol){

    using namespace boost::asio;
    using namespace boost::asio::ip;

    io_service ioService;
    udp::resolver resolver(ioService);
    udp::resolver::query query(host_name(),"");

    std::vector<Interface> interfaces;
    try{
        udp::resolver::iterator it = resolver.resolve(query);

        while(it!=udp::resolver::iterator()){
            address addr =(it++)->endpoint().address();
            if((addr.is_v6() ? Protocol::ipv6 : (addr.is_v4() ? Protocol::ipv4 : Protocol::unknow)) == protocol){
                interfaces.emplace_back(Interface{protocol,addr.to_string()});
            }
        }
    }catch (const boost::system::system_error &error){
        Logger::error(fmt("list_local_interfaces: Cannot list interfaces, error message: {}\n", error.what()));
    }

    ioService.stop();
    return interfaces;
}

std::string Host::get_name(){
    return boost::asio::ip::host_name();
}
