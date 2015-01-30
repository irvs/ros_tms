// Implementation of the ClientSocket class

#include "ClientSocket.h"
#include "SocketException.h"
#include <ros/ros.h>


ClientSocket::ClientSocket ( std::string host, int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create client socket." );
    }
    this->hostIP_s = host;
  if (this->hostIP_s.empty()){
    ROS_ERROR( "host IP enmpty" );
    return;
  }
  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

}


const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
   if (this->hostIP_s.empty()){
    ROS_ERROR( "host IP enmpty" );
    return *this;
  }
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}


const ClientSocket& ClientSocket::operator >> ( std::string& s ) const
{
  if (this->hostIP_s.empty()){
    ROS_ERROR( "host IP enmpty" );
    return *this;
  }
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}
