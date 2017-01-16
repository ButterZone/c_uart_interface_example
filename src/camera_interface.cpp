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
 * @file camera_interface.cpp
 *
 * @brief Camera interface functions
 *
 * Functions for grabbing images from the camera and process the images
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Zihao Wang,     <zwan7346@uni.sydney.edu.au>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "camera_interface.hpp"

// ------------------------------------------------------------------------------
//   Constructor
// ------------------------------------------------------------------------------
Camera_Interface::
Camera_Interface()
{
	// initialize attributes
	camera_reading_status 		= 0; 		// whether the camera read thread is running
	time_to_exit 				= false; 	// flag to signal camera read thread exit
	object_detected 			= false;

	camera_read_tid 			= 0; 		// camera read thread id

	// white background + black object
	// low_hue						= 0;
	// low_saturation				= 0;
	// low_value					= 0;
	// high_hue 					= 255;
	// high_saturation				= 81;
	// high_value 					= 61;

	// office background + green object
	low_hue						= 92;
	low_saturation				= 88;
	low_value					= 53;
	high_hue 					= 125;
	high_saturation 			= 255;
	high_value 					= 193;


	corrected_frame_size_x 		= 0; 		// number of pixels along the x axis
	corrected_frame_size_y 		= 0; 		// number of pixels along the y axis
	centre_of_corrected_frame_x = 0; 		// x coordinate of the frame centre
	centre_of_corrected_frame_y = 0; 		// y coordinate of the frame centre


	track_bar_name_hsv			= "trackbar_hsv";

	radius 						= 0.0f; 	// detected object radius in pixels

	object_actual_diameter 		= 900.0f;	// mm
	focal_length				= 218*200/object_actual_diameter;

	object_offset_horizontal 	= 0;		// offset
	object_offset_vertical 		= 0;
	object_offset_distance 		= 0;

	number_of_centres_to_draw 	= 10;
}

// ------------------------------------------------------------------------------
//   Destructor
// ------------------------------------------------------------------------------
Camera_Interface::
~Camera_Interface()
{}

// ------------------------------------------------------------------------------
//   Check Camera
// ------------------------------------------------------------------------------
void
Camera_Interface::
check_camera() 
{
	if (!Camera.open()) {
		fprintf(stderr, "Error: Failed to open the PiCamera\n");
	}

	return;
}

// ==========================================================================================================
// ==========================================================================================================
// tasks
// ==========================================================================================================
// ==========================================================================================================

// ------------------------------------------------------------------------------
//   Read fame from camera, adjust frame
// ------------------------------------------------------------------------------
void
Camera_Interface::
read_camera() 
{
	// update frame
	Camera.grab();
	Camera.retrieve(original_frame);

	// print error if no frame is captured
	if ( original_frame.empty() ) {
		fprintf(stderr, "Error: Blank frame grabbed\n");
	}

	// reduce orignial frame to a quarter of the size
	resize(original_frame, original_frame, Size(), 0.5, 0.5, INTER_LINEAR);
	// convert PiCamera RGB format to BGR for correct OpenCV expectation
	cvtColor(original_frame, corrected_frame, CV_RGB2BGR);

	return;
}

// ------------------------------------------------------------------------------
//   process the raw frame to capture objects according to HSV range
// ------------------------------------------------------------------------------
void
Camera_Interface::
process_frame_hsv()
{
	// get the size of the frame
	corrected_frame_size_y = corrected_frame.rows;
	corrected_frame_size_x = corrected_frame.cols;

	// find the centre
	centre_of_corrected_frame_x = corrected_frame_size_x/2;
	centre_of_corrected_frame_y = corrected_frame_size_y/2;

	// convert frame into hsv channels
	cvtColor(corrected_frame, processed_frame, CV_RGB2HSV_FULL);

	// detect pixels in the HSV range desired, then erode and dilate to remove small artifacts
	inRange(processed_frame, 
			Scalar(low_hue, low_saturation, low_value), 
			Scalar(high_hue, high_saturation, high_value), 
			processed_frame);
	erode(processed_frame, processed_frame, Mat());
	dilate(processed_frame, processed_frame, Mat());

	// find contours in the frame
	processed_frame.copyTo(contour_frame); // clone this frame for contours
	findContours(contour_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	contour_frame = Mat::zeros(contour_frame.size(), CV_8UC3);

	// iterate through the contour to find the largest contour
	if ( contours.size() > 0 )
	{
		object_detected = true;
		this_contour_area = 0;
		largest_contour_area = 0;
		largest_contour_index = 0;

		for (int i = 0; i < contours.size(); i++)
		{
			this_contour_area = contourArea(contours[i], false);
			if (this_contour_area > largest_contour_area)
			{
				largest_contour_area = this_contour_area;
				largest_contour_index = i;
			} // end if area > largest area
		} // end for all the contours

		// outline the largest contour found
		drawContours( contour_frame, 
						contours, 
						largest_contour_index, 
						Scalar(255,0,0), 
						2, 
						8, 
						hierarchy, 
						0, 
						Point());

		// find the minimum enclosing circle of the largest contour
		minEnclosingCircle(contours[largest_contour_index], centre, radius);

		// draw this circle on the contour frame
		circle(corrected_frame, centre, radius, Scalar(0,255,0), 2);

		// keep track of all the centres in history
		circleCentres.push_back(centre);
	}
	else
	{
		object_detected = false;
	} // end if contours.size() > 0

	// calculate object offsets
	this->calculate_object_offsets();

	// draw the trail of the circle centres if there is enough centres to draw
	if (circleCentres.size() > number_of_centres_to_draw)
	{
		for (int i = 1; i < circleCentres.size(); i++)
		{
			thickness = sqrt(64/(circleCentres.size()-i+1)*2.5); // determine thickness of the tail
			line(contour_frame, circleCentres[i], circleCentres[i-1], Scalar(0,0,255), thickness);
		} // end for all the circle centres

		// keep the vector small
		circleCentres.erase(circleCentres.begin());
	} // end if there is enough circle centres

	// create information stream to be displayed on images
	stringstream hsv_contour_information_line1;
	stringstream hsv_contour_information_line2;
	stringstream hsv_contour_information_line3;
	if (object_detected) 
	{
		hsv_contour_information_line1 	<< "centre: (" << circleCentres.back().x << ","
										<< circleCentres.back().y << ")";
		hsv_contour_information_line2	<< "radius: " << radius;
		hsv_contour_information_line3 	<< "x:" << object_offset_horizontal
										<< " y:" << object_offset_vertical
										<< " z:" << object_offset_distance;
	} 
	else 
	{
		hsv_contour_information_line1 	<< "centre: (no_object)" ;
		hsv_contour_information_line2	<< "radius: 0";
		hsv_contour_information_line3 	<< "x:NaN y:NAN z:NAN";
	}

	hsv_contour_information_string_line1 = hsv_contour_information_line1.str();
	hsv_contour_information_string_line2 = hsv_contour_information_line2.str();
	hsv_contour_information_string_line3 = hsv_contour_information_line3.str();

	// add contours to the corrected frame
	corrected_frame.copyTo(corrected_frame_with_contour);
	corrected_frame_with_contour += contour_frame;

	return;
}

// ------------------------------------------------------------------------------
//   calculate object offsets
// ------------------------------------------------------------------------------
void
Camera_Interface::
calculate_object_offsets()
{
	if ( object_detected )
	{
		// horizontal offsets
		object_offset_horizontal = circleCentres.back().x - centre_of_corrected_frame_x;

		// vertical offsets
		object_offset_vertical = circleCentres.back().y - centre_of_corrected_frame_y;

		// depth
		object_offset_distance = object_actual_diameter*focal_length/radius/2;
	}
	else
	{
		// all offsets are zero if no object detected
		object_offset_horizontal 	= 0;
		object_offset_vertical 		= 0;
		object_offset_distance 		= 0;
	}

	return;
}

// ------------------------------------------------------------------------------
//   calculate yaw target
// ------------------------------------------------------------------------------
void
Camera_Interface::
calculate_yaw_target()
{
	// only calculate when object exists
	if ( contours.size() > 0 )
	{
		yaw_target = object_offset_horizontal/centre_of_corrected_frame_x;
	}
	else
	{
		yaw_target = 0.0f;
	}

	return;
}

// ------------------------------------------------------------------------------
//   calculate horizontal (x) speed target
// ------------------------------------------------------------------------------
void
Camera_Interface::
calculate_x_speed_target()
{
	// keep the object between some distances
	if ( object_offset_distance < 200)
	{
		x_speed_target = -1.0f;
	}
	else if ( object_offset_distance > 300)
	{
		x_speed_target = 1.0f;
	}
	else
	{
		x_speed_target = 0;
	}
	return;
}

// ------------------------------------------------------------------------------
//   calculate depth (y) speed target
// ------------------------------------------------------------------------------
void
Camera_Interface::
calculate_y_speed_target()
{
	// keep the object between some horizontal distances
	if ( object_offset_horizontal > 30 )
	{
		y_speed_target = 1.0f;
	}
	else if ( object_offset_horizontal < -30 )
	{
		y_speed_target = -1.0f;
	}
	else
	{
		y_speed_target = 0;
	}
	return;
}

// ------------------------------------------------------------------------------
//   calculate altitude (z) speed target
// ------------------------------------------------------------------------------
void
Camera_Interface::
calculate_z_speed_target()
{
	// keep the object between some vertical distances
	if ( object_offset_vertical > 30 )
	{
		z_speed_target = 1.0f;
	}
	else if ( object_offset_vertical < -30 )
	{
		z_speed_target = -1.0f;
	}
	else
	{
		z_speed_target = 0;
	}
	return;
}

// ==========================================================================================================
// ==========================================================================================================
// windows
// ==========================================================================================================
// ==========================================================================================================

// ------------------------------------------------------------------------------
//   create trackbar hsv
// ------------------------------------------------------------------------------
void
Camera_Interface::
create_trackbar_hsv()
{
	namedWindow(track_bar_name_hsv);
	createTrackbar("low_hue		   ", track_bar_name_hsv, &low_hue, 255);
	createTrackbar("low_saturation ", track_bar_name_hsv, &low_saturation, 255);
	createTrackbar("low_value      ", track_bar_name_hsv, &low_value, 255);
	createTrackbar("high_hue       ", track_bar_name_hsv, &high_hue, 255);
	createTrackbar("high_saturation", track_bar_name_hsv, &high_saturation, 255);
	createTrackbar("high_value     ", track_bar_name_hsv, &high_value, 255);

	return;
}

// ------------------------------------------------------------------------------
//   Show original frame
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_original_frame() 
{
	imshow( "original", original_frame );
	return;
}

// ------------------------------------------------------------------------------
//   Show corrected frame
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_corrected_frame() 
{
	imshow( "corrected", corrected_frame );
	return;
}

// ------------------------------------------------------------------------------
//   Show corrected frame with contour
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_corrected_frame_with_contour_hsv() 
{
	Size text_size = getTextSize(hsv_contour_information_string_line1, 1, 1, 1, 0);
	Point text_position(2, text_size.height+2);
	Point text_position_line2(2, text_size.height+text_size.height+4);
	Point text_position_line3(2, text_size.height*3 + 6);
	putText(corrected_frame_with_contour,
			hsv_contour_information_string_line1,
			text_position,
			1,
			1,
			Scalar(0, 255, 0),
			2,
			8);
	putText(corrected_frame_with_contour,
			hsv_contour_information_string_line2,
			text_position_line2,
			1,
			1,
			Scalar(0, 255, 0),
			2,
			8);
	putText(corrected_frame_with_contour,
			hsv_contour_information_string_line3,
			text_position_line3,
			1,
			1,
			Scalar(0, 255, 0),
			2,
			8);
	line(corrected_frame_with_contour, 
		Point(centre_of_corrected_frame_x-30, centre_of_corrected_frame_y-30), 
		Point(centre_of_corrected_frame_x+30, centre_of_corrected_frame_y-30),
		Scalar(0,0,255), 1);
	line(corrected_frame_with_contour, 
		Point(centre_of_corrected_frame_x-30, centre_of_corrected_frame_y+30), 
		Point(centre_of_corrected_frame_x+30, centre_of_corrected_frame_y+30),
		Scalar(0,0,255), 1);
	line(corrected_frame_with_contour, 
		Point(centre_of_corrected_frame_x-30, centre_of_corrected_frame_y-30), 
		Point(centre_of_corrected_frame_x-30, centre_of_corrected_frame_y+30),
		Scalar(0,0,255), 1);
	line(corrected_frame_with_contour, 
		Point(centre_of_corrected_frame_x+30, centre_of_corrected_frame_y-30), 
		Point(centre_of_corrected_frame_x+30, centre_of_corrected_frame_y+30),
		Scalar(0,0,255), 1);
	imshow( "corrected with contour", corrected_frame_with_contour );

	return;
}

// ------------------------------------------------------------------------------
//   Show processed frame with trackbar
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_processed_frame_with_trackbar_hsv()
{
	imshow( track_bar_name_hsv, processed_frame );
	return;
}

// ==========================================================================================================
// ==========================================================================================================
// threading
// ==========================================================================================================
// ==========================================================================================================

// ------------------------------------------------------------------------------
//   Startup
// ------------------------------------------------------------------------------
void
Camera_Interface::
start() 
{
	int result;

	printf("START CAMERA READ THREAD\n");

	result = pthread_create( &camera_read_tid, NULL, &start_camera_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading the camera feed
	printf("\n");
	return;
}

// ------------------------------------------------------------------------------
//   Shutdown
// ------------------------------------------------------------------------------
void
Camera_Interface::
stop()
{
	// close camera read thread
	printf("CLOSE CAMERA READ THREAD\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(camera_read_tid, NULL);

	// now the camera read thread is closed
	printf("\n");
}

// ------------------------------------------------------------------------------
//   Start Camera Read Thread
// ------------------------------------------------------------------------------
void
Camera_Interface::
start_camera_read_thread() 
{
	if ( camera_reading_status != 0)
	{
		fprintf(stderr, "camera read thread already running\n");
		return;
	} 
	else 
	{
		camera_read_thread();
		return;
	}
}
// ------------------------------------------------------------------------------
//   Camera Read Thread
// ------------------------------------------------------------------------------
void
Camera_Interface::
camera_read_thread() 
{
	camera_reading_status = true;

	while ( !time_to_exit )
	{
		read_camera();
		usleep(25000); 	// This is roughly 40 Hz
	}

	camera_reading_status = false;

	return;
}

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void
Camera_Interface::
handle_quit( int sig )
{
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr, "Warning, could no stop camera interface\n");
	}
}

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void*
start_camera_interface_read_thread( void *args ) 
{
	// takes a camera interface object argument
	Camera_Interface *camera_interface = (Camera_Interface *)args;

	// run the object's camera read thread
	camera_interface->start_camera_read_thread();

	// done!
	return NULL;
}