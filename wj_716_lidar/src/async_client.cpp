#include "async_client.h"

Async_Client::Async_Client(boost::asio::io_service& io_service,ip::tcp::endpoint endpoint,fundata_t fundata_ )
  :iosev(io_service),
    socket(iosev),
    ep(endpoint),
    m_connected(0),
    m_fundata(fundata_)
{
  socket.connect(ep,ec);
  if(ec)
  {
    std::cout << boost::system::system_error(ec).what() << std::endl;
    m_connected = 0 ;
  }
  else
  {
    cout<<" Connection Success! "<<ep.address().to_string()<<endl;
    m_connected = 1 ;
  }
  client_async_read();
}
void Async_Client::client_async_write(char *buf,int len)
{
  boost::asio::async_write(socket,
                           boost::asio::buffer(buf, len),
                           boost::bind(&Async_Client::handle_write, this,
                                       boost::asio::placeholders::error));
}
void Async_Client::client_async_read()
{
  socket.async_read_some(boost::asio::buffer(data_, MAX_LENGTH),
                         boost::bind(&Async_Client::handle_read, this,
                                     boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}
bool Async_Client::client_return_status(){
  return m_connected ;
}


void Async_Client::handle_read(const boost::system::error_code& error,
                               size_t bytes_transferred)
{
  if (!error)
  {
    //printf("From %s Received data Len:%d\n",this->ep.address().to_string().c_str(),
          // bytes_transferred);
    (*m_fundata)(this->ep.address().to_string().c_str(),this->ep.port(),data_,bytes_transferred);
    this->client_async_read();
  }
  else
  {
    printf("%s\n",error.message().c_str());
    m_connected = 0 ;
  }
}

void Async_Client::handle_write(const boost::system::error_code& error)
{
  if (!error)
  {

  }
  else
  {
    printf("%s\n",error.message().c_str());
    m_connected = 0 ;
  }
}

void Async_Client::socket_close()
{

  socket.close();

}

