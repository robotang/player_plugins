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
 
#ifndef SLAM_GMAPPING_H
#define SLAM_GMAPPING_H

#include <stdint.h>
#include <libplayercore/playercore.h>
#include <pthread.h>

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#define GMAPPING_MAP_OCCUPIED        1
#define GMAPPING_MAP_FREE        -1
#define GMAPPING_MAP_UNKNOWN        0

using namespace std;

class slam_gmapping
{
public:
    slam_gmapping();
    ~slam_gmapping();

    bool init(ConfigFile *cf, int section, player_position2d_geom_t *initial_pose, player_laser_geom_t *laser_geom, player_laser_config_t *laser_config);
    bool update(player_laser_data_t *laser_scan, player_position2d_data_t *pose);
    void update_map(void);
    bool get_map(player_map_data_t *mapreq, player_map_data_t *mapresp);
    bool get_map_info(player_map_info_t *map_info);
    bool get_pose(player_position2d_data_t *pose);

private:
    bool init_mapper(player_laser_data_t *laser_scan, player_position2d_data_t *pose);    
    
    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    GMapping::OdometrySensor* gsp_odom_;
    double gsp_laser_angle_resolution_;
    double gsp_laser_min_angle_;
    int gsp_laser_ranges_count_;
    bool got_first_scan_;
    GMapping::OrientedPoint* laser_pose_;
    bool got_map_;
    player_map_data_t map_;
    
    pthread_t map_thread_;
    bool update_map_;
    pthread_mutex_t map_mutex_;
        
    //Laser parameters
    double maxRange_; //maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
    double maxUrange_; //maximum range of the laser scanner that is used for map building (default: same as maxRange)
    double minUrange_;
    double sigma_; //standard deviation for the scan matching process (cell)
    int kernelSize_; //search window for the scan matching process
    double lstep_; //initial search step for scan matching (linear)
    double astep_; //initial search step for scan matching (angular)
    int iterations_; //number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
    double lsigma_; //standard deviation for the scan matching process (single laser beam)
    double ogain_; //gain for smoothing the likelihood
    int lskip_; //take only every (n+1)th laser ray for computing a match (0 = take all rays)
    
    //Motion Model Parameters (all standard deviations of a gaussian noise model)
    double srr_; //linear noise component (x and y)
    double srt_; //linear -> angular noise component
    double str_; //angular -> linear noise component
    double stt_; //angular noise component (theta)
    
    //Likelihood sampling (used in scan matching)
    double llsamplerange_;
    double lasamplerange_;
    double llsamplestep_;
    double lasamplestep_;
    
    //Initial map dimensions and resolution (all in metres)
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_; //Map resolution    
    
    //Others
    double linearUpdate_; //the robot only processes new measurements if the robot has moved at least this many meters
    double angularUpdate_; //the robot only processes new measurements if the robot has turned at least this many rads
    double resampleThreshold_;
    int particles_; //number of particles. Each particle represents a possible trajectory that the robot has traveled    
    double temporalUpdate_; //threshold at which the particles get resampled. Higher means more frequent resampling
    double occ_threshold_;    
};

#endif

