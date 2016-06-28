/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


/**********************************************************************
 * $Author:: MomenTUM $  $Date:: 2015-03-15 13:29:48#$ $Rev:: 26104  $*
 **********************************************************************/

/*! \brief definition of constants used by MTUM filters
 */

#ifndef _MOMENTUM_CONST_H_
#define _MOMENTUM_CONST_H_

//general
#define DEBUG                   //remove define to disable debug functions
#define PI 3.14159265358979323846
#define DEG2RAD PI/180
#define RAD2DEG 180/PI


//grid defaults
#define TILE_COUNT_LEFT 150//120//40
#define TILE_COUNT_RIGHT 100//60//20
#define TILE_COUNT_FRONT 200//200//100
#define TILE_COUNT_REAR 60//100//50
#define GRID_CM_PER_UNIT 1          //size of grid cell in cm
#define CARSIZE_CROSS 30            //car size cross in cm
#define CARSIZE_DRIVING 60          //car size driving direction in cm
#define CARSIZE_CENTER_TO_FRONT 49  //distance from car base to front in cm
#define CARSIZE_CENTER_TO_REAR 11  //distance from car base to rear in cm
#define CARSIZE_AXISDIST 36.4      //distance between front and rearaxis in cm
#define GRID_BASE_VALUE 128
#define MIN_OBSTACLE_LENGTH 10     // minimal obstacle length in grid cells
#define SENSOR_SHORT_THRESHOLD 250  //short sensor value to be ignored if sensor is combined with a long sensor
#define SENSOR_LONG_THRESHOLD 200   //long range senorvalues to be ignored if bigger

#define IR_MAX_DIFF 10.0            //distance two combined ir sensors are allowed to differ in cm

//motion estimator
#define UPDATE_THRESHOLD 1         //sensor values needed to calc new motion data set
#define WHEEL_RADIUS  5            //wheel raidus in cm
#define DRIVING_DIR_THRESHOLD 0.25  //threshold to differ between front/backwards and stand
//#define WHEEL_TICKS_TO_DIST 3.927  //wheelradius * 2 * pi / 8
#define WHEEL_TICKS_PER_ROUND 8
#define GYRO_EPSILON 0.00001        //gyroscope angle change to be treated as 0

//depth sensor settings
#define DEPTH_BASE_OFFSET 250            //value an object needs to differ from ground value to be reconized as object
#define DEPTH_IGNORE_THRESHOLD 10     //higest value to be ignored as invalid (black areas)
#define DEPTH_IGNORE_HEIGHT_RATIO 0.35   //percent of the depth image rows to be ignored
#define DEPTH_IGNORE_RIGHT_SIDE 0   //pixels of the depth image most right cols to be ignored
#define DEPTH_IGNORE_LEFT_SIDE 0   //pixels of the depth image most left cols to be ignored
#define DEPTH_IGNORE_ROWS_DOWN 78
#define DEPTH_IGNORE_ROWS_UP 24//64
#define INVALID_COLINDEX_MAX 56     //value calculated from black pixels to mark column as invalid

// utile
#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

//decision maker
#define SEGMENT_ROT_THRESHOLD 0.5   //rotation difference between soll and ist
#define MAX_ACC 35                  //maximun acceleration value
#define MAX_TESTING_RANGE 200      //grid units to be tested for driving
#define TEST_SAMPLE_POINT_COUNT 10  //number of sample points per arc of testing area
#define VIEW_MAX_SPEED_DIST 170     //view distance to be max speed in cm
#define TEST_MAX_ERROR_VALUE 8000	//value to be threated as obstacle
#define TEST_REF_GREY_VALUE 168		//values above are being ignored (threated as free)
#define PARKING_TESTING_RANGE 10        //range to bested while parking in a parallel parking lot; reduced speed till /2 at < --> stop
#define SPEED_BASE_VALUE 25.0			//normal base speed to let the car drive
#define SPEED_MIN_VALUE 20.0            //min speed to not thread as 0
#define SPEED_BACKWARDS -25.0          //constanct speed to be used for driving backwards
#define CURVATURE_SPEED_REDUCTION_RATIO 0.7 //ratio to have speed reduced to while driving curves
#define BREAK_COUNTER_START 15		//start value of the break counter (1=break for 1/30 fps)
#define TEST_FIRST_AREA_LENGTH 20   //driving direction size of the close area obstacle testing area

#define MIN_DIST2OBST_BEFORE_OVERTAKING 57  //trying to start overtaking if distance to obstale on current lane is smaler than defined
#define OVERTAKING_TEST_RANGE 120   //range to check in cm before starting overtaking
#define HIGHSPEED_MIN_DIST 150.0      //dist in cm to be driven before highspeed is being enabled

//lanedefinition
#define LANE_WIDTH 44           //width of one lane in cm
#define LANE_CENTER_EPSILON 1   //difference from lane middle until correction takes place (in cm)
#define CURVATURE_SMALL_EPSILON 0.00005
#define MAX_STEERING_ANGLE 28


enum states {STATE_IDLE=1,STATE_ENABLED,STATE_LANEFOLLOWING,STATE_DRIVESEGMENTS,STATE_PARKING};
enum actions {ACTION_STOP=0,ACTION_LEFT=1,ACTION_STRAIGHT=2,ACTION_RIGHT=3,ACTION_CROSS_PARKING=4,ACTION_PARALLEL_PARKING=5,ACTION_PULL_OUT_LEFT=6,ACTION_PULL_OUT_RIGHT=7,ACTION_PULL_OUT_RIGHT_CP=8,ACTION_PULL_OUT_RIGHT_PP=9};
enum calcErrModes {USE_BOTH = 0, USE_DEPTH_ONLY,USE_SENSOR_ONLY};



#endif // _MOMENTUM_CONST_H_
