#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <unistd.h>

using namespace std ;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using namespace boost::asio;
#define MAX_LENGTH 50000
typedef void (*fundata_t)(const char* addr,int port,const char* data,const int len);
class Async_Client
{
public:
  Async_Client();
  Async_Client(boost::asio::io_service& io_service,ip::tcp::endpoint endpoint,fundata_t fundata_ );
  void client_async_write(char *buf,int len);
  void client_async_read();
  bool client_return_status();

  void socket_close(void);

private:
  void handle_read(const boost::system::error_code& error,
                   size_t bytes_transferred);

  void handle_write(const boost::system::error_code& error);



  io_service &iosev;
  ip::tcp::socket socket ;
  ip::tcp::endpoint ep ;
  boost::system::error_code ec;

  char        data_[MAX_LENGTH];
  bool        m_connected ;
  fundata_t   m_fundata ;
};

#endif // ASYNC_CLIENT_H
