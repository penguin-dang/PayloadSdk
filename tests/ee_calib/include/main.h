#ifndef __MAIN_H
#define __MAIN_H

#include "log.h"
#include "payloadSdkInterface.h"
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <chrono>

#if defined GHADRON
#include "ghadron_sdk.h"
#elif defined VIO
#include "vio_sdk.h"
#elif defined ZIO
#include "zio_sdk.h"
#endif
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <thread>
#include "common.h"
#include "pid.h"
PID *pidTILT,*pidPAN;

/**/
PayloadSdkInterface* my_payload = nullptr;

bool time_to_exit = false;
/*!< Private prototype*/
static uint64_t _get_time_usec(){
 	auto currentTime = std::chrono::high_resolution_clock::now();
 	auto duration = currentTime.time_since_epoch();
 	return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}
static uint64_t _get_time_msec(){
	auto currentTime = std::chrono::system_clock::now();
	auto duration = currentTime.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}
/*!< Private typedef*/
typedef enum{
	STATE_IDLE = 0,
	STATE_ERROR,
	STATE_WAIT_TO_CONNECT_PAYLOAD,
	STATE_SET_PAYLOAD_PARAM,
	STATE_LOAD_PAYLOAD_PARAM,
	STATE_START_MOVEMENT,
	STATE_MOVEMENT_0,
	STATE_MOVEMENT_1,
	STATE_MOVEMENT_2,
	STATE_MOVEMENT_3,
	STATE_MOVEMENT_4,
	STATE_MOVEMENT_5,
	STATE_MOVEMENT_6,
	STATE_MOVEMENT_7,
	STATE_MOVEMENT_8,
	STATE_DONE,
} E_sample_process_state;

const char* state_name[STATE_DONE + 1] = {
	"STATE_IDLE = 0",
	"STATE_ERROR",
	"STATE_WAIT_TO_CONNECT_PAYLOAD",
	"STATE_SET_PAYLOAD_PARAM",
	"STATE_LOAD_PAYLOAD_PARAM",
	"STATE_START_MOVEMENT",
	"STATE_MOVEMENT_0",
	"STATE_MOVEMENT_1",
	"STATE_MOVEMENT_2",
	"STATE_MOVEMENT_3",
	"STATE_MOVEMENT_4",
	"STATE_MOVEMENT_5",
	"STATE_MOVEMENT_6",
	"STATE_MOVEMENT_7",
	"STATE_MOVEMENT_8",
	"STATE_DONE"	
};

typedef struct{
	E_sample_process_state _state = STATE_IDLE;
	uint64_t _time_usec = _get_time_usec();
} T_psdk_process_state;

void onPayloadStatusChanged(int event, double* param);

// Init PID
void initializePID();

// Global variable to store the trackbar value
int EO_zoom_value = 0;

// Frame Queue
#include <queue>
#include <mutex>
#include <condition_variable>
std::queue<cv::Mat> frameQueue;
std::mutex queueMutex;
std::condition_variable queueCondVar;
bool stopProcessing = false;
const int MAX_QUEUE_SIZE = 5;
std::vector<cv::Point2f> corners; //this will be filled by the detected corners
bool patternfound;

// CALIBRATION
bool pan_calib = false, x_home = false, y_home = false;
long long total_time; // in seconds
void getTotalTime();
std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

#endif 