/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

/* Adapted from costmap_2d/costmap_2d.h */

#include <limits.h>
#include <cmath>
#include <stdlib.h>
#include <algorithm>
#include "channel_controller/bresenham.h"

namespace channel_controller
{

Bresenham::Bresenham(int x0, int y0, int x1, int y1, unsigned int max_length)
{
    cur_x_ = x0;
    cur_y_ = y0;
    dx = x1 - x0;
    dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    //if x is dominant
    if(abs_dx >= abs_dy){
        abs_da = abs_dx;
        abs_db = abs_dy;
        go_x = true;
    } else {
        //otherwise y is dominant
        abs_da = abs_dy;
        abs_db = abs_dx;
        go_x = false;
    }
    error_b = abs_da / 2;

    i = 0;
    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double scale = std::min(1.0,  max_length / dist);
    end = std::min((unsigned int)(scale * abs_da), abs_da);
}

bool Bresenham::hasNext() const
{
    return i <= end;
}

void Bresenham::advance()
{
    if(go_x) {
        cur_x_ += sign(dx);
    } else {
        cur_y_ += sign(dy);
    }
    error_b += abs_db;
    if((unsigned int)error_b >= abs_da){
        if(go_x) {
            cur_y_ += sign(dy);
        } else {
            cur_x_ += sign(dx);
        }
        error_b -= abs_da;
    }
    i++;
}

}

