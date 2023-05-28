/*
***********************************************************************************************************************************************************************
This file includes imported packages.
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

#if BUILD_TYPE == BUILD_TYPE_DEBUG
    #define DEBUG_PRINT
#else
    #undef DEBUG_PRINT
#endif

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iomanip>
#include <memory>
#include <string>
#include <map>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/SetModelState.h"
#include "UDP_Socket.hpp"

using namespace std::chrono;
