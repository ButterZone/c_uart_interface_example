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
check_camera() {
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
read_camera() {
	// update frame
	Camera.grab();
	Camera.retrieve(frame);

	// print error if no frame is captured
	if ( frame.empty() ) {
		fprintf(stderr, "Error: Blank frame grabbed\n");
	}

	// reduce orignial frame to a quarter of the size
	resize(frame, frame, Size(), 0.5, 0.5, INTER_LINEAR);
	// convert PiCamera RGB format to BGR for correct OpenCV expectation
	cvtColor(frame, frame, CV_RGB2BGR);

	return;
}

// ------------------------------------------------------------------------------
//   Show original frame
// ------------------------------------------------------------------------------
void
Camera_Interface::
show_original_frame() {
	imshow( "original", frame );
	return;
}

// ------------------------------------------------------------------------------
//   Startup
// ------------------------------------------------------------------------------
void
Camera_Interface::
start() {
	printf("START CAMERA \n");
	return;
}