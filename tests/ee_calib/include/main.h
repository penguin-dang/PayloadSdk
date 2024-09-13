#ifndef __MAIN_H
#define __MAIN_H

#include "log.h"
#include "payloadSdkInterface.h"
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <chrono>
#include <fstream> // for file stream

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

const int frame_width = 1920;
const int frame_height = 1080;

using namespace cv;
using namespace std;

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
	STATE_WAIT_TO_CONNECT_PAYLOAD,
	STATE_SET_PAYLOAD_PARAM,
	STATE_DATA_WROTE,
	STATE_HOME,
	STATE_NEXT_ANGLE,
	STATE_START_PAN,
	STATE_START_TILT,
	STATE_CENTERING,
	STATE_DONE,
} E_sample_process_state;

const char* state_name[STATE_DONE + 1] = {
	"STATE_IDLE = 0",
	"STATE_WAIT_TO_CONNECT_PAYLOAD",
	"STATE_SET_PAYLOAD_PARAM",
	"STATE_DATA_WROTE",
	"STATE_HOME",
	"STATE_NEXT_ANGLE",
	"STATE_START_PAN",
	"STATE_START_TILT",
	"STATE_CENTERING",
	"STATE_DONE"	
};

typedef struct{
	E_sample_process_state _state = STATE_IDLE;
	uint64_t _time_usec = _get_time_usec();
} T_psdk_process_state;

void onPayloadStatusChanged(int event, double* param);

// UI
// Set color, status for btns
void setBtnStatus();
// Define postion of btns
void definePositionBtns(cv::Mat &frame);

// Init PID
void initializePID();

// Global variable to store the trackbar value
int EO_zoom_value = 0;

// Create an output file stream object
std::ofstream outFilePan("calibarationPAN.txt");
std::ofstream outFileTilt("calibarationTILT.txt");

int proccessCalibration();

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
int chessBoardX = 0;
int chessBoardY = 0;

// CALIBRATION
bool tilt_en = false, pan_en = false, x_home = false, y_home = false;
long long total_time; // in seconds
void getTotalTime();
double getWidthPixelFromCenter(double px_x);
double getHeightPixelFromCenter(double px_y);
std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
auto skip_startTime = std::chrono::steady_clock::now();
float imu_yaw = 0.0, imu_pitch = 0.0;
float send_yaw = 0.0, send_pitch = 0.0;
float error_px = 0.0; // Pixel error from center of Chessboard to center frame
float next_angle_send = 0.0;

float starting_yaw = 0.0;
float starting_pitch = 0.0;

int sharpnessValue = 0; // Initial sharpness value

int mouse_move_x = 0;
int mouse_move_y = 0;

bool touch = false;
int touchX = 0;
int touchY = 0;

bool ready_click = false;

#endif 