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
double kp_, kd_, ki_;
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
            if (pan_en) {
                s_proc._state = STATE_CENTERING;
            }
        }
        break;
    case STATE_CENTERING:
        {
            s_proc._state = STATE_DATA_WROTE;
        }
        break;
    case STATE_DATA_WROTE:
        {
            if (patternfound) {
                
                s_proc._state = STATE_DATA_WROTE;
            }
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
    // std::cout << "Enter kp, kd, ki ";
    // std::cin >> kp_ >> kd_ >> ki_;

    // pidPAN->set_kp((double)kp_);
    // pidPAN->set_kd((double)kd_);
    // pidPAN->set_ki((double)ki_);
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
    // if (abs(errorX) < 1) errorX = 0;
    // if (abs(errorY) < 1) errorY = 0;

    double errorTILT = pidTILT->get_error();
    double errorPAN = pidPAN->get_error();

    double speed_pitch  = 0;
    double speed_roll   = 0;
    double speed_yaw    = 0;

    // Compute PID control signals for tilt and pan axes
    if (errorX != 0) {
        speed_yaw    = -pidPAN->calculate(0, errorX);
        // x_home = false;
        // start_time = std::chrono::steady_clock::now();
        // total_time = 0;
    }
    else {
        speed_yaw = 0;
        // x_home = true;
    }

    if (errorY != 0) {
        speed_pitch  = pidTILT->calculate(0, errorY);
        // y_home = false;
        // start_time = std::chrono::steady_clock::now();
        // total_time = 0;
    }
    else {
        speed_pitch = 0;
        // y_home = true;
    }
    // printf("speed_yaw %f, speed_pitch %f\n", speed_yaw, speed_pitch);
    my_payload->setGimbalSpeed(speed_pitch, 0 , speed_yaw, INPUT_SPEED);
    return 0;
}

// Call this function to get the total time x_home was true
void getTotalTime() {
    auto end_time = std::chrono::steady_clock::now();
    total_time += std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    // std::cout << " total_time HOME: " << total_time << std::endl;
}

double getWidthPixelFromCenter(double px_x) {
    return px_x - frame_width/2;
}

double getHeightPixelFromCenter(double px_y) {
    return px_y - frame_height/2;
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
Button btn_left, btn_down, btn_up, btn_right, btn_pan, btn_tilt, btn_lock, btn_follow, btn_return;
bool move_left = false, move_down = false, move_right = false, move_up = false;
bool en_touch = false, en_track = false, en_detection = false;
cv::Scalar btn_left_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_down_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_up_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_right_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_pan_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_tilt_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_lock_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_follow_color = cv::Scalar(255, 0, 0);  // Initially blue
cv::Scalar btn_return_color = cv::Scalar(255, 0, 0);  // Initially blue

void definePositionBtns(cv::Mat &frame) {
    // Calculate button sizes and positions based on frame size
    int buttonWidth = 100;
    int buttonHeight = 50;
    int padding = 10;
    int startX = frame.cols - buttonWidth - padding;
    int startY = frame.rows - buttonHeight - padding;

    // Define button rectangles in the bottom-right corner
    btn_left = {cv::Rect(startX - 2*(buttonWidth + padding), startY, buttonWidth, buttonHeight), "LEFT"};
    btn_down = {cv::Rect(startX - 1*(buttonWidth + padding), startY, buttonWidth, buttonHeight), "DOWN"};
    btn_up = {cv::Rect(startX - 1*(buttonWidth + padding),
            startY - 1*(buttonHeight + padding),
            buttonWidth, buttonHeight),
            "UP"};
    btn_right = {cv::Rect(startX, startY, buttonWidth, buttonHeight), "RIGHT"};

    btn_pan = {cv::Rect(startX - 0.3*buttonWidth, startY - 3*(buttonHeight + padding), 
            1.3*buttonWidth, buttonHeight), "PAN Calb"};
    btn_tilt = {cv::Rect(startX - 0.3*buttonWidth, startY - 4*(buttonHeight + padding), 
            1.3*buttonWidth, buttonHeight), "TILT Calb"};

    btn_lock   = {cv::Rect(startX - 0.3*buttonWidth, startY - 6*(buttonHeight + padding), 
            1.3*buttonWidth, buttonHeight), "LOCK"};
    btn_follow = {cv::Rect(startX - 0.3*buttonWidth, startY - 7*(buttonHeight + padding), 
            1.3*buttonWidth, buttonHeight), "FOLLOW"};
    btn_return = {cv::Rect(startX - 0.3*buttonWidth, startY - 8*(buttonHeight + padding), 
            1.3*buttonWidth, buttonHeight), "RETURN"};

    // Draw buttons on the frame
    setBtnStatus();
    cv::rectangle(frame, btn_left.rect, btn_left_color, -1);
    cv::rectangle(frame, btn_down.rect, btn_down_color, -1);
    cv::rectangle(frame, btn_up.rect, btn_up_color, -1);
    cv::rectangle(frame, btn_right.rect, btn_right_color, -1);
    cv::rectangle(frame, btn_pan.rect, btn_pan_color, -1);
    cv::rectangle(frame, btn_tilt.rect, btn_tilt_color, -1);
    cv::rectangle(frame, btn_lock.rect, btn_lock_color, -1);
    cv::rectangle(frame, btn_follow.rect, btn_follow_color, -1);
    cv::rectangle(frame, btn_return.rect, btn_return_color, -1);

    // Draw labels on the buttonsq
    cv::putText(frame, btn_left.label, btn_left.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_down.label, btn_down.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_up.label, btn_up.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_right.label, btn_right.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_pan.label, btn_pan.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_tilt.label, btn_tilt.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_lock.label, btn_lock.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_follow.label, btn_follow.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    cv::putText(frame, btn_return.label, btn_return.rect.tl() + cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}
 
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
            pan_en = !pan_en;
            next_angle_send = 0.0;
            starting_yaw = imu_yaw;
            starting_pitch = imu_pitch;
            if (!pan_en)
                s_proc._state = STATE_IDLE;
            std::cout << "Button PAN clicked, status: " << pan_en << std::endl;
        } else if (btn_tilt.isClicked(clickPoint)) {
            std::cout << "Button TILT clicked"  << std::endl;
        } else if (btn_lock.isClicked(clickPoint)) {
            std::cout << "Button LOCK clicked"  << std::endl;
        	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, 
                    PAYLOAD_CAMERA_GIMBAL_MODE_LOCK, PARAM_TYPE_UINT32);
        } else if (btn_follow.isClicked(clickPoint)) {
            std::cout << "Button FOLLOW clicked"  << std::endl;
        	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, 
                    PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
        } else if (btn_return.isClicked(clickPoint)) {
            std::cout << "Button RETURN clicked"  << std::endl;
        	my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, 
                    PAYLOAD_CAMERA_GIMBAL_MODE_RESET, PARAM_TYPE_UINT32);
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
        if (proccessCalibration() < 0) {
            break;
        }
        usleep(30000);  // Sleep for 1ms
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
        // printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
        imu_pitch = param[0];
        imu_yaw = param[2];
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

int proccessCalibration() {
    int count = 0;
    switch(s_proc._state){
    case STATE_IDLE:
        {
            if (pan_en) {
                s_proc._state = STATE_CENTERING;
            }
        }
        break;
    case STATE_CENTERING:
        {
            my_payload->setGimbalSpeed(0, starting_pitch, starting_yaw, INPUT_ANGLE);
            usleep(100000);
            my_payload->setGimbalSpeed(0, starting_pitch, starting_yaw, INPUT_ANGLE);
            usleep(100000);
            my_payload->setGimbalSpeed(0, starting_pitch, starting_yaw, INPUT_ANGLE);
            usleep(2000000);
            errorX = getWidthPixelFromCenter(corners[0].x);
            std::cout << " Write Center -> " << errorX << " " << imu_yaw << std::endl;
            outFile.open("calibaration.txt", std::ios::app);
            outFile << "Center: " << errorX << " " << imu_yaw << std::endl;
            outFile.close();
            sleep(1);
            s_proc._state = STATE_NEXT_ANGLE;
            // std::cout << " errorX " << errorX << std::endl;
            // if (abs(errorX) < 30) {
            //     auto skip_currentTime = std::chrono::steady_clock::now();
            //     auto skip_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(skip_currentTime - skip_startTime);
            //     if (skip_elapsed > std::chrono::milliseconds(3000)) {
            //         std::cout << " Write Center -> " << errorX << " " << imu_yaw << std::endl;
            //         outFile.open("calibaration.txt", std::ios::app);
            //         outFile << "Center: " << errorX << " " << imu_yaw << std::endl;
            //         outFile.close();
            //         sleep(1);
            //         s_proc._state = STATE_NEXT_ANGLE;
            //     } else {
            //         cal_gimbal_speed(1.0, static_cast<float>(corners[0].x), static_cast<float>(corners[0].y));
            //     }

            // } else {
            //     cal_gimbal_speed(1.0, static_cast<float>(corners[0].x), static_cast<float>(corners[0].y));
            //     skip_startTime = std::chrono::steady_clock::now();
            // }
        }
        break;
    case STATE_NEXT_ANGLE:
        {
            next_angle_send+=1.0;
            float yaw_send = starting_yaw + next_angle_send;
            my_payload->setGimbalSpeed(0, starting_pitch, yaw_send, INPUT_ANGLE);
            usleep(100000);
            my_payload->setGimbalSpeed(0, starting_pitch, yaw_send, INPUT_ANGLE);
            usleep(100000);
            my_payload->setGimbalSpeed(0, starting_pitch, yaw_send, INPUT_ANGLE);
            usleep(2000000);
            errorX = getWidthPixelFromCenter(corners[0].x);
            std::cout << " Write Target -> " << errorX << " " << imu_yaw << std::endl;
            outFile.open("calibaration.txt", std::ios::app);
            outFile << "Target: " << errorX << " " << imu_yaw << std::endl;
            outFile.close();
            sleep(1);
            if (count > 10)  {
                q = false;
            } else {
                s_proc._state = STATE_CENTERING;
            }
            count+=1;
        }
        break;
    default:
        break;
    }
    // if (pan_en) {
    //     errorX = getWidthPixelFromCenter(corners[0].x);
    //     if (abs(errorX) < 30) {
    //         auto skip_currentTime = std::chrono::steady_clock::now();
    //         auto skip_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(skip_currentTime - skip_startTime);
    //         if (skip_elapsed > std::chrono::milliseconds(3000)) {
    //             if (outFile.is_open()) {
    //                 outFile << errorX;
    //                 outFile << imu_yaw;
    //             }

    //         } else {
    //             cal_gimbal_speed(1.0, static_cast<float>(corners[0].x), static_cast<float>(corners[0].y));
    //         }

    //     } else {
    //         cal_gimbal_speed(1.0, static_cast<float>(corners[0].x), static_cast<float>(corners[0].y));
    //         skip_startTime = std::chrono::steady_clock::now();
    //     }
    // }
    return 1;
}

// Function to initialize the PID controllers
void initializePID() {
    pidTILT = new PID(0.03, 100, -100, 0.04, 0.01, 0.0);
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

        definePositionBtns(frame);
        if(patternfound) {
            chessBoardX = static_cast<int>(corners[0].x - 0.5);
            chessBoardY = static_cast<int>(corners[0].y - 0.5);
            cv::circle(frame,
                    cv::Point(chessBoardX, chessBoardY), 
                    4, 
                    cv::Scalar(255, 0, 0), 
                    4);
            // std::cout << " corners[22].x " << corners[22].x << " corners[22].y " << corners[22].y << std::endl;
        }
        // proccessCalibration();

        // Get the center coordinates
        int centerX = frame.cols / 2;
        int centerY = frame.rows / 2;

        // Draw a horizontal line through the center
        cv::line(frame, cv::Point(0, centerY), cv::Point(frame.cols, centerY), cv::Scalar(0, 0, 255), 1);
        // Draw a vertical line through the center
        cv::line(frame, cv::Point(centerX, 0), cv::Point(centerX, frame.rows), cv::Scalar(0, 0, 255), 1);


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