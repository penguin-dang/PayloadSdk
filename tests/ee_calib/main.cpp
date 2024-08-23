#include "main.h"

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

enum tracking_cmd_t{
    TRACK_IDLE = 0,
    TRACK_ACT = 1
};

T_psdk_process_state s_proc;

void quit_handler(int sig){
    printf("\n");
    printf("TERMINATING AT USER REQUEST \n");
    printf("\n");

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    exit(0);
}

/*!<@brief:
 * @para1:
 * @retval:
 * */
int8_t psdk_run_sample(){
    mavlink_message_t msg;
    if(my_payload == nullptr) {
        PRINT_ERR("%s | %d | PayloadSdkInterface is nullptr");
        return -1;
    }
    uint8_t msg_cnt = my_payload->getNewMewssage(msg);
    if(!msg_cnt){
        return 1;
    }

    switch(s_proc._state){
    case STATE_IDLE:
        {
            s_proc._state = STATE_WAIT_TO_CONNECT_PAYLOAD;
        }
        break;
    case STATE_WAIT_TO_CONNECT_PAYLOAD:
        {
            if(msg.sysid == PAYLOAD_SYSTEM_ID && msg.compid == PAYLOAD_COMPONENT_ID){	// found message from payload
                
                PRINT_INFO("%s | %s",__func__,state_name[s_proc._state]);
                PRINT_INFO("Connnected to payload!!!");

                usleep(5000000);
                s_proc._state = STATE_SET_PAYLOAD_PARAM;
            }
        }
        break;
    default:
        break;
    }
    return 1;
}

double targetX = 1920/2;
double targetY = 1080/2;
int cal_gimbal_speed(double _score, double x, double y) {
    // Update the currentX and currentY variables accordingly.
    if (_score < 0.5) {
        my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
        return 0;
    }

    // Compute errors
    double errorX = x - targetX;
    double errorY = y - targetY;

    // PID deadzone
    if (abs(errorX) < 5) errorX = 0;
    if (abs(errorY) < 5) errorY = 0;

    double errorTILT = pidTILT->get_error();
    double errorPAN = pidPAN->get_error();

    double speed_pitch  = 0;
    double speed_roll   = 0;
    double speed_yaw    = 0;

    // Compute PID control signals for tilt and pan axes
    if (errorX != 0) {
        speed_yaw    = -pidPAN->calculate(0, errorX);
        x_home = false;
        start_time = std::chrono::steady_clock::now();
        total_time = 0;
    }
    else {
        speed_yaw = 0;
        x_home = true;
    }

    if (errorY != 0) {
        speed_pitch  = pidTILT->calculate(0, errorY);
        y_home = false;
        start_time = std::chrono::steady_clock::now();
        total_time = 0;
    }
    else {
        speed_pitch = 0;
        y_home = true;
    }
    // printf("speed_yaw %f, speed_pitch %f\n", speed_yaw, speed_pitch);
    my_payload->setGimbalSpeed(speed_pitch, 0 , speed_yaw, INPUT_SPEED);
    return 0;
}

// Call this function to get the total time x_home was true
void getTotalTime() {
    auto end_time = std::chrono::steady_clock::now();
    total_time += std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    return total_time;
}

// Button properties
struct Button {
    cv::Rect rect;
    std::string label;
    bool isClicked(const cv::Point &pt) const {
        return rect.contains(pt);
    }
};

// Global variables
Button btn_left, btn_down, btn_up, btn_right, btn_pan, btn_tilt;
bool move_left = false, move_down = false, move_right = false, move_up = false;
bool en_touch = false, en_track = false, en_detection = false;
cv::Scalar btn_left_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_down_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_up_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_right_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_pan_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_tilt_color = cv::Scalar(255, 0, 0);  // Initially blue

// Mouse callback function
void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point clickPoint(x, y);
        if (btn_left.isClicked(clickPoint)) {
            move_left = !move_left;
            std::cout << "Button LEFT clicked" << std::endl;
            if (move_left)
                my_payload->setGimbalSpeed(0, 0 , -5, INPUT_SPEED);
            else
                my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
        } else if (btn_down.isClicked(clickPoint)) {
            move_down = !move_down;
            std::cout << "Button DOWN clicked" << std::endl;
            if (move_down)
                my_payload->setGimbalSpeed(-5, 0 , 0, INPUT_SPEED);
            else
                my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
        } else if (btn_right.isClicked(clickPoint)) {
            move_right = !move_right;
            std::cout << "Button RIGHT clicked " << std::endl;
            if (move_right)
                my_payload->setGimbalSpeed(0, 0 , 5, INPUT_SPEED);
            else
                my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
        } else if (btn_up.isClicked(clickPoint)) {
            move_up = !move_up;
            std::cout << "Button UP clicked"  << std::endl;
            if (move_up)
                my_payload->setGimbalSpeed(5, 0 , 0, INPUT_SPEED);
            else
                my_payload->setGimbalSpeed(0, 0 , 0, INPUT_SPEED);
        } else if (btn_pan.isClicked(clickPoint)) {
            std::cout << "Button PAN clicked"  << std::endl;
            pan_calib = true;
        } else if (btn_tilt.isClicked(clickPoint)) {
            std::cout << "Button TILT clicked"  << std::endl;
        } else {
            std::cout << "Send TOUCH position (en_track, x, y): (" << en_track << ", " << x << ", " << y << ")" << std::endl;
        }
    }
}

// Callback function for the trackbar (if needed)
void on_trackbar_EO_zoom(int, void*) {
    std::cout << "EO zoom value: " << EO_zoom_value+1 << std::endl;
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR, EO_zoom_value, PARAM_TYPE_UINT32);

}

void setBtnStatus() {
    if (en_touch) {
        btn_left_color = cv::Scalar(0, 255, 0);  // Change color to green
    } else {
        btn_left_color = cv::Scalar(255, 0, 0);  // Change color to blue
    }
    if (en_track) {
        btn_down_color = cv::Scalar(0, 255, 0);  // Change color to green
    } else {
        btn_down_color = cv::Scalar(255, 0, 0);  // Change color to blue
    }
    if (en_detection) {
        btn_right_color = cv::Scalar(0, 255, 0);  // Change color to green
    } else {
        btn_right_color = cv::Scalar(255, 0, 0);  // Change color to blue
    }
}

void run_in_thread() {
    while (!time_to_exit) {
        if (psdk_run_sample() < 0) {
            break;
        }
        usleep(1000);  // Sleep for 1ms
    }
    std::cout << "Thread exiting..." << std::endl;
}

void onPayloadStatusChanged(int event, double* param){
    switch(event){
    case PAYLOAD_GB_ACK:{
        // param[0]: command
        // param[1]: resutl
        // isRecivedAck = true;
        std::cout << "Got ACK for command " << param[0] << " with result " << param[1] << std::endl;
        break;
    }
    case PAYLOAD_GB_ATTITUDE:{
        // param[0]: pitch
        // param[1]: roll
        // param[2]: yaw

        printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
        break;
    }
    default: break;
    }
}

void processFrame() {
    while (true) {
        if (!frameQueue.empty()) {
            // break;
            cv::Mat frame = frameQueue.front();
            frameQueue.pop();

            cv::Size patternsize(9,6); //interior number of corners
            cv::Mat gray_tmp;
            cv::cvtColor(frame,gray_tmp,cv::COLOR_BGR2GRAY);//source image
            // int up_width = 640;
            // int up_height = 480;
            // cv::resize(gray_tmp, gray_tmp, cv::Size(up_width, up_height), cv::INTER_LINEAR);
            cv::Mat gray = cv::Scalar::all(255) - gray_tmp;


            // that do not contain any chessboard corners
            patternfound = cv::findChessboardCorners(gray, patternsize, corners,
                                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                + cv::CALIB_CB_FAST_CHECK);

            if(patternfound) {
                cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            }
        }
        usleep(10000);
    }
}

// Function to initialize the PID controllers
void initializePID() {
    pidTILT = new PID(0.03, 100, -100, 0.06, 0.01, 0.0);
    pidPAN = new PID(0.03, 100, -100, 0.05, 0.01, 0.0);
}

int main(int argc,char** argv){
    signal(SIGINT,quit_handler);
    time_to_exit = false;
    /*!Init payload interface class pointer*/
    my_payload = new PayloadSdkInterface(s_conn);
    initializePID();
    if(my_payload->sdkInitConnection() == false){
        PRINT_ERR("Payload Interface Init failed,close the program!!!!");
        exit(1);
    }

    printf("Set gimbal RC mode \n");
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);

    // register callback function
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

    // Start the thread
    std::thread worker(run_in_thread);

    // Define the GStreamer pipeline string
    std::string pipeline = "rtspsrc latency=0 location=rtsp://192.168.12.217:8554/ghadron ! decodebin ! videoconvert ! appsink";

    // Create a VideoCapture object with the GStreamer pipeline
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    // Check if the stream was opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the stream." << std::endl;
        return -1;
    }

    // Create a window to display the video
    cv::namedWindow("RTSP Stream", cv::WINDOW_NORMAL);
    // Set the initial size of the window to 1280x720
    cv::resizeWindow("RTSP Stream", 1280, 720);
    cv::setMouseCallback("RTSP Stream", onMouse);
    // Create a trackbar in the same window
    // cv::createTrackbar("EO Zoom", "RTSP Stream", &EO_zoom_value, 11, on_trackbar_EO_zoom);

    // Loop to continuously get frames from the stream
    cv::Mat frame;
    std::thread processingThread(processFrame);
    while (true) {
        if (!cap.read(frame)) {
            std::cerr << "Error: Could not read frame." << std::endl;
            break;
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (frameQueue.size() >= MAX_QUEUE_SIZE) {
                // Clear the entire queue if it exceeds the maximum size
                std::queue<cv::Mat> empty;
                std::swap(frameQueue, empty);
                // std::cout << "Queue cleared due to overflow." << std::endl;
            }
            frameQueue.push(frame.clone());
        }
        queueCondVar.notify_one();

        // Calculate button sizes and positions based on frame size
        int buttonWidth = 100;
        int buttonHeight = 50;
        int padding = 10;
        int startX = frame.cols - buttonWidth - padding;
        int startY = frame.rows - buttonHeight - padding;

        // Define button rectangles in the bottom-right corner
        btn_left = {cv::Rect(startX - 2 * (buttonWidth + padding), startY, buttonWidth, buttonHeight), "LEFT"};
        btn_down = {cv::Rect(startX - 1 * (buttonWidth + padding), startY, buttonWidth, buttonHeight), "DOWN"};
        btn_up = {cv::Rect(startX - 1 * (buttonWidth + padding), startY - 1 * (buttonHeight + padding), buttonWidth, buttonHeight), "UP"};
        btn_right = {cv::Rect(startX, startY, buttonWidth, buttonHeight), "RIGHT"};


        // Draw buttons on the frame
        setBtnStatus();
        cv::rectangle(frame, btn_left.rect, btn_left_color, -1);
        cv::rectangle(frame, btn_down.rect, btn_down_color, -1);
        cv::rectangle(frame, btn_up.rect, btn_up_color, -1);
        cv::rectangle(frame, btn_right.rect, btn_right_color, -1);

        // Draw labels on the buttonsq
        cv::putText(frame, btn_left.label, btn_left.rect.tl() + cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(frame, btn_down.label, btn_down.rect.tl() + cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(frame, btn_up.label, btn_up.rect.tl() + cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(frame, btn_right.label, btn_right.rect.tl() + cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        if(patternfound && pan_calib) {
            cv::circle(frame,
                    cv::Point(static_cast<int>(corners[22].x - 0.5), static_cast<int>(corners[22].y - 0.5)), 
                    4, 
                    cv::Scalar(255, 0, 0), 
                    4);
            cal_gimbal_speed(1.0, static_cast<float>(corners[22].x), static_cast<float>(corners[22].y));
        } else {
            cal_gimbal_speed(0.0, 0.0, 0.0);
        }
        getTotalTime();
        if (pan_calib && )

        // cv::drawChessboardCorners(frame, patternsize, cv::Mat(corners), qpatternfound);

        // Display the frame
        cv::imshow("RTSP Stream", frame);

        // Exit the loop if 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            printf("\n");
            printf("TERMINATING AT USER REQUEST \n");
            printf("\n");

            // close payload interface
            try {
                my_payload->sdkQuit();
            }
            catch (int error){}
            break;
        }
        // Signal the processing thread to stop
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            stopProcessing = true;
        }
        // queueCondVar.notify_all();
        // processingThread.join();
    }

    // Release the VideoCapture object and close the display window
    cap.release();
    cv::destroyAllWindows();

    return 0;
}