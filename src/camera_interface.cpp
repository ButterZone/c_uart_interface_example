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
	camera_reading_status = 0; 		// whether the camera read thread is running
	time_to_exit 	= false; 		// flag to signal camera read thread exit

	camera_read_tid = 0; 					// camera read thread id

	low_hue			= 0;
	low_saturation	= 0;
	low_value		= 0;
	high_hue 		= 255;
	high_saturation = 255;
	high_value 		= 255;

	trackBarName 	= "trackbar";

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

void
Camera_Interface::
process_frame()
{
	// convert frame into hsv channels
	cvtColor(corrected_frame, processed_frame, CV_RGB2HSV_FULL);
	// detect pixels in the HSV range desired
	inRange(processed_frame, Scalar(low_hue, low_saturation, low_value), Scalar(high_hue, high_saturation, high_value), processed_frame);
	erode(processed_frame, processed_frame, Mat());
	dilate(processed_frame, processed_frame, Mat());
}

void
Camera_Interface::
create_trackbar()
{
	namedWindow(trackBarName);
	createTrackbar("low_hue		   ", trackBarName, &low_hue, 255);
	createTrackbar("low_saturation ", trackBarName, &low_saturation, 255);
	createTrackbar("low_value      ", trackBarName, &low_value, 255);
	createTrackbar("high_hue       ", trackBarName, &high_hue, 255);
	createTrackbar("high_saturation", trackBarName, &high_saturation, 255);
	createTrackbar("high_value     ", trackBarName, &high_value, 255);

	return;
}

// ------------------------------------------------------------------------------
//   Show original frame
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_original_frame() 
{
	imshow( "original", corrected_frame );
	return;
}

// ------------------------------------------------------------------------------
//   Show processed frame with trackbar
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_processed_frame_with_trackbar()
{
	imshow( trackBarName, processed_frame );
	return;
}

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