/*
 *  Copyright (C) 2011, Robert Tang <opensource@robotang.co.nz>
 *
 *  This is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public Licence
 *  along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef LASER2D_FILTER_H
#define LASER2D_FILTER_H

#include <stdint.h>

#define LASER2D_FILTER_NONE        0
#define LASER2D_FILTER_MEDIAN      (1 << 0) //1
#define LASER2D_FILTER_EDGE        (1 << 1) //2
#define LASER2D_FILTER_RANGE       (1 << 2) //4
#define LASER2D_FILTER_MEAN        (1 << 3) //8
#define LASER2D_FILTER_FILL        (1 << 4) //16

using namespace std;

class laser2d_filter
{
public:
    laser2d_filter(int length_, bool wrap_around_, int filter_type_, int filter_depth_);
    ~laser2d_filter();
    
    void process(float *data);
private:
    void process_fill(float *data);
    
    int length;
    bool wrap_around;
    int filter_type;
    int filter_depth;
    
    float *last_valid;
    int *last_valid_counter;
};

#endif

