/*
 *  A wrapper for interfacing gmapping to Player (ported from ROS gmapping wrapper) 
 *
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
#include <math.h>
#include <time.h>
#include <string.h>

#include "slam_gmapping.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

//#define DEBUG_SLAM_GMAPPING

//Laser parameters
#define DEFAULT_MAXRANGE        8.0
#define DEFAULT_MAXURANGE        DEFAULT_MAXRANGE
#define DEFAULT_MINURANGE        0.1
#define DEFAULT_SIGMA            0.05
#define DEFAULT_KERNELSIZE        1
#define DEFAULT_LSTEP            0.05
#define DEFAULT_ASTEP            0.05
#define DEFAULT_ITERATIONS        5
#define DEFAULT_LSIGMA            0.075
#define DEFAULT_OGAIN            3.0
#define DEFAULT_LSKIP            0

//Motion Model Parameters (all standard deviations of a gaussian noise model)
#define DEFAULT_SRR            0.1
#define DEFAULT_STT            0.2
#define DEFAULT_SRT            0.2
#define DEFAULT_STR            0.1

//Likelihood sampling (used in scan matching)
#define DEFAULT_LLSAMPLERANGE        0.01
#define DEFAULT_LASAMPLERANGE        0.005
#define DEFAULT_LLSAMPLESTEP        0.01
#define DEFAULT_LASAMPLESTEP        0.005

//Initial map dimensions and resolution parameters (all in metres)
#define DEFAULT_XMIN            -15
#define DEFAULT_YMIN            -15
#define DEFAULT_XMAX            15
#define DEFAULT_YMAX            15
#define DEFAULT_DELTA            0.05

//Others
#define DEFAULT_LINEARUPDATE        1.0
#define DEFAULT_ANGULARUPDATE        0.5
#define DEFAULT_RESAMPLETHRESHOLD    0.5
#define DEFAULT_PARTICLES        30
#define DEFAULT_TEMPORALUPDATE        0.1//-1.0 //-ive -> does not update map based on period
#define DEFAULT_OCCTHRESHOLD        0.25

void *update_map_callback(void *obj)
{
    static_cast<slam_gmapping*>(obj)->update_map();
    return 0;
}

slam_gmapping::slam_gmapping()
{
    gsp_ = NULL;
    gsp_laser_ = NULL;
    gsp_odom_ = NULL;
    got_first_scan_ = false;
    laser_pose_ = NULL;
    got_map_ = false;
    map_.data = NULL;
    update_map_ = false;
    
    pthread_mutex_init(&map_mutex_, 0);
    pthread_create(&map_thread_, 0, &update_map_callback, this);
}

slam_gmapping::~slam_gmapping()
{
    if(gsp_)
        delete gsp_;
    if(gsp_laser_)
        delete gsp_laser_;
    if(gsp_odom_)
        delete gsp_odom_;
    if(laser_pose_)
        delete laser_pose_;
    if(map_.data)
        delete map_.data;

    pthread_exit(&map_thread_);
    pthread_mutex_destroy(&map_mutex_);
}

bool slam_gmapping::init(ConfigFile *cf, int section, player_position2d_geom_t *initial_pose, player_laser_geom_t *laser_geom, player_laser_config_t *laser_config)
{
    bool ret = true;
    
    //#ifdef DEBUG_SLAM_GMAPPING
    printf("odom geom - px: %f, py: %f, pyaw: %f\r\n", initial_pose->pose.px, initial_pose->pose.py, initial_pose->pose.pyaw);
    printf("laser geom - px: %f, py: %f, pyaw: %f\r\n", laser_geom->pose.px, laser_geom->pose.py, laser_geom->pose.pyaw); 
    printf("laser config - min_angle: %f, max_angle: %f, resolution: %f, max_range: %f, range_res: %f, intensity: %d, scanning frequency: %f\r\n", 
        laser_config->min_angle, laser_config->max_angle, laser_config->resolution, laser_config->max_range,
        laser_config->range_res, laser_config->intensity, laser_config->scanning_frequency);
    //#endif
    
    //Set laser parameters
    maxRange_ = laser_config->max_range - 0.01; //cf->ReadFloat(section, "maxRange", DEFAULT_MAXRANGE);
    maxUrange_ = maxRange_; //cf->ReadFloat(section, "maxUrange", DEFAULT_MAXURANGE);
    minUrange_ = cf->ReadFloat(section, "maxUrange", DEFAULT_MINURANGE);
    sigma_ = cf->ReadFloat(section, "sigma", DEFAULT_SIGMA);
    kernelSize_ = cf->ReadInt(section, "kernelSize", DEFAULT_KERNELSIZE);
    lstep_ = cf->ReadFloat(section, "lstep", DEFAULT_LSTEP);
    astep_ = cf->ReadFloat(section, "astep", DEFAULT_ASTEP);
    iterations_ = cf->ReadInt(section, "iterations", DEFAULT_ITERATIONS);
    lsigma_ = cf->ReadFloat(section, "lsigma", DEFAULT_LSIGMA);
    ogain_ = cf->ReadFloat(section, "ogain", DEFAULT_OGAIN);
    lskip_ = cf->ReadInt(section, "lskip", DEFAULT_LSKIP);    
    
    //Set motion model parameters
    srr_ = cf->ReadFloat(section, "srr", DEFAULT_SRR);
    stt_ = cf->ReadFloat(section, "stt", DEFAULT_STT);
    srt_ = cf->ReadFloat(section, "srt", DEFAULT_SRT);
    str_ = cf->ReadFloat(section, "str", DEFAULT_STR);    

    //Set likelihood sampling parameters
    llsamplerange_ = cf->ReadFloat(section, "llsamplerange", DEFAULT_LLSAMPLERANGE);
    lasamplerange_ = cf->ReadFloat(section, "lasamplerange", DEFAULT_LASAMPLERANGE);
    llsamplestep_ = cf->ReadFloat(section, "llsamplestep", DEFAULT_LLSAMPLESTEP);        
    lasamplestep_ = cf->ReadFloat(section, "lasamplestep", DEFAULT_LASAMPLESTEP);

    //Set initial map dimensions and resolution
    xmin_ = cf->ReadFloat(section, "xmin", DEFAULT_XMIN);
    ymin_ = cf->ReadFloat(section, "ymin", DEFAULT_YMIN);
    xmax_ = cf->ReadFloat(section, "xmax", DEFAULT_XMAX);
    ymax_ = cf->ReadFloat(section, "ymax", DEFAULT_YMAX);
    delta_ = cf->ReadFloat(section, "delta", DEFAULT_DELTA);    

    //Set other parameters
    linearUpdate_ = cf->ReadFloat(section, "linearUpdate", DEFAULT_LINEARUPDATE);
    angularUpdate_ = cf->ReadFloat(section, "angularUpdate", DEFAULT_ANGULARUPDATE);
    resampleThreshold_ = cf->ReadFloat(section, "resampleThreshold", DEFAULT_RESAMPLETHRESHOLD);
    particles_ = cf->ReadInt(section, "particles", DEFAULT_PARTICLES);
    temporalUpdate_ = cf->ReadFloat(section, "temporalUpdate", DEFAULT_TEMPORALUPDATE);
    occ_threshold_ = cf->ReadFloat(section, "occ_threshold", DEFAULT_OCCTHRESHOLD);
    
    //Create GMapping processor
    gsp_ = new GMapping::GridSlamProcessor();
    assert(gsp_);

    laser_pose_ = new GMapping::OrientedPoint(laser_geom->pose.px, laser_geom->pose.py, laser_geom->pose.pyaw);
    //printf("\t\t\t\tlaser pose - x: %f, y: %f, yaw: %f\r\n", laser_geom->pose.px, laser_geom->pose.py, laser_geom->pose.pyaw);

    //printf("xmin: %f, xmax: %f, ymin: %f, ymax: %f\r\n", xmin_, xmax_, ymin_, ymax_);
    //printf("odom: %f, %f, %f\r\n", initial_pose->pose.px, initial_pose->pose.py, initial_pose->pose.pyaw);
    //Increase the size of the map if initial pose is close to a boundary
    if(xmin_ > (initial_pose->pose.px - 1.0)) xmin_ = initial_pose->pose.px - 1.0;
    else if(xmax_ < (initial_pose->pose.px + 1.0)) xmax_ = initial_pose->pose.px + 1.0;
    if(ymin_ > (initial_pose->pose.py - 1.0)) ymin_ = initial_pose->pose.py - 1.0;
    else if(ymax_ < (initial_pose->pose.py + 1.0)) ymax_ = initial_pose->pose.py + 1.0;
    
    //printf("xmin: %f, xmax: %f, ymin: %f, ymax: %f\r\n", xmin_, xmax_, ymin_, ymax_);

    return ret;
}

bool slam_gmapping::init_mapper(player_laser_data_t *laser_scan, player_position2d_data_t *pose)
{
    bool ret = true;
    
    //Create GMapping laser module    
    gsp_laser_angle_resolution_ = laser_scan->resolution;
    gsp_laser_ranges_count_ = laser_scan->ranges_count;
    gsp_laser_min_angle_ = laser_scan->min_angle;
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_ranges_count_, gsp_laser_angle_resolution_, *laser_pose_, 0.0, maxRange_);
    assert(gsp_laser_);
    
    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);
    
    //Create GMapping odometry module
    gsp_odom_ = new GMapping::OdometrySensor("odom");
    assert(gsp_odom_);
    
    GMapping::OrientedPoint initialPose(pose->pos.px, pose->pos.py, pose->pos.pa);
    
    //Setup GMapping
    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, ogain_, lskip_);
    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    
    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    
    GMapping::sampleGaussian(1,time(NULL));
    
    return ret;
}

bool slam_gmapping::update(player_laser_data_t *laser_scan, player_position2d_data_t *pose)
{    
    if(!got_first_scan_)
    {
        init_mapper(laser_scan, pose);
        got_first_scan_ = true;
    }
    
    //Pre-process laser data
    double *ranges = new double [laser_scan->ranges_count];
    for(int i = 0; i < laser_scan->ranges_count; i++)
    {
        ranges[i] = laser_scan->ranges[i];
        if(ranges[i] > maxUrange_) ranges[i] = maxUrange_;
        if(ranges[i] < minUrange_) ranges[i] = minUrange_;
    }
    
    //Add laser data, set pose, and cleanup
    GMapping::RangeReading reading(laser_scan->ranges_count, ranges, gsp_laser_);
    delete [] ranges;
    GMapping::OrientedPoint gmap_pose(pose->pos.px, pose->pos.py, pose->pos.pa);
    //printf("\t\t\t\tpose - x: %f, y: %f, yaw: %f\r\n", pose->pos.px, pose->pos.py, pose->pos.pa);
    reading.setPose(gmap_pose);
    
    //Process scan and return result
    bool processed = gsp_->processScan(reading);
    
    //Update map if required
    if(!got_map_ || processed)
    {
        pthread_mutex_lock(&map_mutex_);
        update_map_ = true;
        pthread_mutex_unlock(&map_mutex_);
    }
    
    return processed;
}

void slam_gmapping::update_map(void)
{
    player_map_data_t map_tmp;
    map_tmp.data = NULL;
    
    while(true)
    {    
        if(!update_map_)
        {
            usleep(100);
            continue;
        }
    
        GMapping::ScanMatcher matcher;
        double *laser_angles = new double[gsp_laser_ranges_count_];
        double theta = gsp_laser_min_angle_;
        for(int i = 0; i < gsp_laser_ranges_count_; i++)
        {
            laser_angles[i] = theta;
            theta += gsp_laser_angle_resolution_;
        }
    
        matcher.setLaserParameters(gsp_laser_ranges_count_, laser_angles, gsp_laser_->getPose());
        delete [] laser_angles;
        matcher.setlaserMaxRange(maxRange_);
        matcher.setusableRange(maxUrange_);
        matcher.setgenerateMap(true);

        GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

        GMapping::Point centre;
        centre.x = (xmin_ + xmax_) / 2.0;
        centre.y = (ymin_ + ymax_) / 2.0;

        GMapping::ScanMatcherMap smap(centre, xmin_, ymin_, xmax_, ymax_, delta_);

        for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
        {
            if(n->reading)
            {
                matcher.invalidateActiveArea();
                matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
                matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
            }
        }
    
        //Check if the map has not been initialised or has been resized
        if(!got_map_ || map_tmp.width != smap.getMapSizeX() || map_tmp.height != smap.getMapSizeY())
        {            
            double width = abs(xmin_) + xmax_, height = abs(ymin_) + ymax_;
            GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
            GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
            xmin_ = wmin.x; ymin_ = wmin.y; xmax_ = wmax.x; ymax_ = wmax.y;
            
            map_tmp.width = smap.getMapSizeX();
            map_tmp.height = smap.getMapSizeY();
            map_tmp.data_count = map_tmp.width * map_tmp.height;
            if(map_tmp.data == NULL)
            {
                map_tmp.data = new int8_t[map_tmp.data_count];
            }
            else //resize map data
            {
                delete [] map_tmp.data;
                map_tmp.data = new int8_t[map_tmp.data_count];
            }
        }

        //For each pixel in smap, copy thresholded values to map_tmp 
        for(int x = 0; x < map_tmp.width; x++)
        {
            for(int y = 0; y < map_tmp.height; y++)
            {
                double occ = smap.cell(x, y);
                      assert(occ <= 1.0);
                      
                      if (occ < 0) map_tmp.data[x + y * map_tmp.width] = GMAPPING_MAP_UNKNOWN;
                      else if (occ > occ_threshold_) map_tmp.data[x + y * map_tmp.width] = GMAPPING_MAP_OCCUPIED;
                      else map_tmp.data[x + y * map_tmp.width] = GMAPPING_MAP_FREE;
            }
        }
    
        //Copy map_tmp over to map_
        pthread_mutex_lock(&map_mutex_);
        if(map_.width != map_tmp.width || map_.height != map_tmp.height)
        {
            map_.width = map_tmp.width;
            map_.height = map_tmp.height;
            map_.data_count = map_tmp.data_count;
            if(map_.data == NULL)
            {
                map_.data = new int8_t[map_.data_count];
            }
            else //resize map data
            {
                delete [] map_.data;
                map_.data = new int8_t[map_.data_count];
            }
        }
        memcpy(map_.data, map_tmp.data, sizeof(int8_t) * map_.data_count);
        got_map_ = true;
        update_map_ = false;
        pthread_mutex_unlock(&map_mutex_);
    }
}

bool slam_gmapping::get_map(player_map_data_t *mapreq, player_map_data_t *mapresp)
{
    if(got_map_ && map_.width && map_.height)
    {
        pthread_mutex_lock(&map_mutex_);
        mapresp->col = mapreq->col;
        mapresp->row = mapreq->row;
        mapresp->width = mapreq->width;
        mapresp->height = mapreq->height;
        mapresp->data_count = mapresp->width * mapresp->height;
        mapresp->data = new int8_t [mapresp->data_count];
        mapresp->data_range = 1;
        
        if(mapreq->width == map_.width && mapreq->height == map_.height)
        {
            /* Map is small enough to be copied in one tile */
            memcpy(mapresp->data, map_.data, sizeof(int8_t) * mapresp->data_count);        
        }
        else
        {
            /* Map needs to be copied in tiles */
            for(int row = mapresp->row, i = 0; row < mapresp->row + mapresp->height; row++, i++)
            {
                memcpy(&mapresp->data[i * mapresp->width], &map_.data[row * map_.width + mapresp->col], sizeof(int8_t) * mapresp->width);
            }
        }
        
        pthread_mutex_unlock(&map_mutex_);
        return true;
    }
    else
        return false;
}

bool slam_gmapping::get_map_info(player_map_info_t *map_info)
{
    bool ret = true;
    
    map_info->scale = delta_;
    map_info->width = map_.width;
    map_info->height = map_.height;
    map_info->origin.px = xmin_; //FIXME
    map_info->origin.py = ymin_; //FIXME
    map_info->origin.pa = 0.0; //FIXME
    
    return ret;
}

bool slam_gmapping::get_pose(player_position2d_data_t *pose)
{
    bool ret = true;
    
    if(got_first_scan_)
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
        pose->pos.px = mpose.x - ((xmin_ + xmax_)/2.0);
        pose->pos.py = mpose.y - ((ymin_ + ymax_)/2.0);
        pose->pos.pa = mpose.theta;
        //Confine theta to -M_PI and M_PI
        while(fabs(pose->pos.pa) > M_PI)
        {
            if(pose->pos.pa > 2*M_PI) pose->pos.pa -= 2*M_PI;
            else if(pose->pos.pa < -2*M_PI) pose->pos.pa += 2*M_PI;
            else if(pose->pos.pa < -1*M_PI) pose->pos.pa += 2*M_PI;
            else /*if(pose->pos.pa > M_PI)*/ pose->pos.pa -= 2*M_PI;                
        }
    }
    else
    {
        pose->pos.px = 0.0;
        pose->pos.py = 0.0;
        pose->pos.pa = 0.0;
    }
    
    pose->vel.px = 0.0; //FIXME
    pose->vel.py = 0.0; //FIXME
    pose->vel.pa = 0.0; //FIXME
    pose->stall = 0; //FIXME
    
    return ret;
}

