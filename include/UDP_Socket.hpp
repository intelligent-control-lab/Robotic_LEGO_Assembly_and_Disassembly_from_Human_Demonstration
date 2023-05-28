/*
***********************************************************************************************************************************************************************
This file defines the UDP socket for controlling the robot.
Copyright (C) 2023

Authors:
Ruixuan Liu: ruixuanl@andrew.cmu.edu
Changliu Liu : cliu6@andrew.cmu.edu

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************************************************************************************************************
*/

#pragma once
#include "Utils/Common.hpp"

namespace lego_assembly
{
namespace udp
{
class UDP_Socket
{
    public:
        typedef std::shared_ptr<UDP_Socket> Ptr;
        typedef std::shared_ptr<UDP_Socket const> ConstPtr;

    public:
        UDP_Socket()
        {
        }
        ~UDP_Socket()
        {
        }

        void Setup(const std::string& IP_addr, unsigned short port, unsigned int timeout_us)  
        {
            try{
                if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
                { 
                    perror("UDP socket creation failed!"); 
                    exit(EXIT_FAILURE);
                } 
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = inet_addr(IP_addr.c_str());
                addr.sin_port = htons(port);
                
                tv.tv_sec = 0;
                tv.tv_usec = timeout_us;
                setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
                addr_len = sizeof(addr);
            }
            catch(...){
                close(sockfd);
                exit(EXIT_FAILURE);
            }
        }
        

        void SendTo(const void* buffer, int len)
        {
            int ret = sendto(sockfd, buffer, len, 0, (sockaddr*)&addr, addr_len);
            if(ret < 0)
            {
                printf("UDP socket send failed!"); 
            }
        }

        void RecvFrom(void* buffer, int len)
        {
            int ret = recvfrom(sockfd, buffer, len, 0, (sockaddr*)&addr, &addr_len);
            if (ret < 0)
            {
                printf("UDP socket recv failed!"); 
            }
        }

        void Close(){
            close(sockfd);
        }

    private:
        int sockfd;
        sockaddr_in addr;
        struct timeval tv;
        socklen_t addr_len;
};
}
}