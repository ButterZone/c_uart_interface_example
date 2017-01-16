/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file camera_interface.hpp
 *
 * @brief Camera interface definition
 *
 * Functions for grabbing images from the camera and process the images
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Zihao Wang,     <zwan7346@uni.sydney.edu.au>
 *
 */

#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_

#include "raspicam/raspicam_cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <fcntl.h>

using namespace cv;
using namespace std;

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------
void* start_camera_interface_read_thread(void *args);

// ----------------------------------------------------------------------------------
//   Camera Interface Class
// ----------------------------------------------------------------------------------
/*
 * Camera Interface Class
 *
 * 
 */
class Camera_Interface
{

public:

	Camera_Interface();
	~Camera_Interface();

	char camera_reading_status;

	bool object_detected;

	int low_hue;
	int low_saturation;
	int low_value;
	int high_hue;
	int high_saturation;
	int high_value;

	int corrected_frame_size_x;
	int corrected_frame_size_y;
	int centre_of_corrected_frame_x;
	int centre_of_corrected_frame_y;

	string track_bar_name_hsv;

	float object_actual_diameter;
	float focal_length;

	float object_offset_horizontal;
	float object_offset_vertical;
	float object_offset_distance;

	float yaw_target;
	float x_speed_target;
	float y_speed_target;
	float z_speed_target;

	void check_camera();
	void read_camera();
	void process_frame_hsv();
	void calculate_object_offsets();

	void calculate_yaw_target();
	void calculate_x_speed_target();
	void calculate_y_speed_target();
	void calculate_z_speed_target();

	void create_trackbar_hsv();
	void show_original_frame();
	void show_corrected_frame();
	void show_corrected_frame_with_contour_hsv();
	void show_processed_frame_with_trackbar_hsv();

	void start();
	void stop();

	void start_camera_read_thread();

	void handle_quit( int sig );

private:

	// camera
	raspicam::RaspiCam_Cv Camera;

	// frames
	Mat original_frame;
	Mat corrected_frame;
	Mat processed_frame;
	Mat contour_frame;
	Mat corrected_frame_with_contour;

	// arrays
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point2f> circleCentres;

	// contours
	double this_contour_area;
	double largest_contour_area;
	int largest_contour_index;
	float radius;
	Point2f centre;
	int thickness;
	int number_of_centres_to_draw;

	// information
	string hsv_contour_information_string_line1;
	string hsv_contour_information_string_line2;
	string hsv_contour_information_string_line3;

	bool time_to_exit;

	pthread_t camera_read_tid;

	void camera_read_thread();
};

#endif // CAMERA_INTERFACE_H_