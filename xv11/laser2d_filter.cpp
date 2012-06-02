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
 
#include <iostream>
#include <stdio.h>

#include "laser2d_filter.h"

laser2d_filter::laser2d_filter(int length_, bool wrap_around_, int filter_type_, int filter_depth_)
{
    length = length_;
    wrap_around = wrap_around_;
    filter_type = filter_type_;
    filter_depth = filter_depth_;
    
    last_valid = new float[length];
    last_valid_counter = new int[length];
    
    for(int i = 0; i < length; i++)
    {
        last_valid[i] = 0.0;
        last_valid_counter[i] = 0;
    }
}

laser2d_filter::~laser2d_filter()
{
    if(last_valid) delete [] last_valid;
    if(last_valid_counter) delete [] last_valid_counter;
}

void laser2d_filter::process(float *data)
{
    /*if(filter_type == LASER2D_FILTER_MEDIAN)
    {
        cout << "laser2d median filter unimplemented" << endl;
    }
    
    if(filter_type == LASER2D_FILTER_EDGE)
    {
        cout << "laser2d edge filter unimplemented" << endl;
    }
    
    if(filter_type == LASER2D_FILTER_RANGE)
    {
        cout << "laser2d range filter unimplemented" << endl;
    }

    if(filter_type == LASER2D_FILTER_MEAN)
    {
        cout << "laser2d mean filter unimplemented" << endl;    
    }*/

    if(filter_type == LASER2D_FILTER_FILL)
    {
        process_fill(data);
    }
}

void laser2d_filter::process_fill(float *data)
{
    for(int i = 0; i < length; i++)
    {
        if(data[i] < 0.0)
        {
            if(last_valid_counter[i] < filter_depth)
            {
                data[i] = last_valid[i];
                last_valid_counter[i]++;            
            }
            else
                last_valid[i] = -1.0; //invalid range
        }    
        else
        {
            last_valid[i] = data[i];
            last_valid_counter[i] = 0;
        }
    }
}

/*void laser2d_filter::process_fill(float *data, int length, bool wrap_around)
{
    int index = 1;    
    while(index < length - 1)
    {
        int start = index;
        for( ; data[start] > 0.0 && start < length; start++) { ; } //find index of first bad value
        start -= 1;
        for(index = start + 1; data[index] < 0.0 && index < length; index++) { ; } //find index of next bad value
        
        //fill in bad values with an approximate linear interpolation
        float increment = (data[index] - data[start]) / (index - start);
        for(int i = start, j = 1; i < index; i++, j++)
            data[i] = data[start] + j*increment;
    }
    
    if(wrap_around)
    {
        ;
    }
}*/
