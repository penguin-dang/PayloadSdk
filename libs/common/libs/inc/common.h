#ifndef _COMMON_H_
#define _COMMON_H_

#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string>

#include <ctime>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <sstream>

#define ICT (+7)

/**
 * Payload storage struct
 * Use to syncronize between CameraManager and GstreamerInterface
 **/
struct StorageInfo {
    uint8_t storage_id;
    uint8_t storage_count;
    uint8_t status;
    float total_capacity;
    float used_capacity;
    float available_capacity;
    float read_speed;
    float write_speed;

    StorageInfo(){
        defaultValue();
    }
    void defaultValue(){
        storage_id = 0;
        storage_count = 1;
        status = 0;
        total_capacity = 0;
        used_capacity = 0;
        available_capacity = 0;
        read_speed = 200;
        write_speed = 90;
    }
};

/**
 * Payload view mode
 * Use to syncronize between CameraManager and GstreamerInterface
 **/
enum videoSource_t{
    vid_pipEO_IR = 0,
    vid_EO_Only,
    vid_IR_Only,
    vid_pipIR_EO,
    vid_pipSync,
    vid_pipOSD,
    vid_currentRecording
};

/**
 * Thermal calib mode
 * Use to disable overlay in camera thermal
 **/
enum statusThermalOverlay{
    THERMAL_OVERLAY_DISABLE = 0,
    THERMAL_OVERLAY_ENABLE
};

/**
 * video stream command
 * pass-throught from CameraManager to GstreamerInterface
 **/ 
enum mavcam_command_t{
    MAVCAM_START_RECORD_VIDEO = 0,
    MAVCAM_STOP_RECORD_VIDEO,
    MAVCAM_START_IMAGE_CAPTURE,
    MAVCAM_STOP_IMAGE_CAPTURE,
    MAVCAM_VIDEO_SRC_CHANGE,
    MAVCAM_VIDEO_REC_CHANGE,
    MAVCAM_EO_ZOOM_CHANGE,
    MAVCAM_EO_ZOOM_STOP,
    MAVCAM_EO_ZOOM_START,
    MAVCAM_IR_ZOOM_CHANGE,
    MAVCAM_TRACKING_TRIGGER,
    MAVCAM_CAMERA_CONNECTION_STATUS,
    MAVCAM_GIMBAL_CHANGE_MODE,
    MAVCAM_TRACKING_CHANGE_PID,
    MAVCAM_CAM_VI_OUTPUT,
    MAVCAM_SMART_TRACKING_STATUS,
    MAVCAM_CAMERA_EO_OSD_MODE,
    MAVCAM_DETECTION_TRIGGER,
    MAVCAM_DO_OSD_ML_ENABLE,
    MAVCAM_RC_MODE,
    MAVCAM_MAPPING_STATE,
    MAVCAM_GIMBAL_FT_ZOOM,
    MAVCAM_STORAGE, 
    MAVCAM_THERMAL_CALIB_MODE,
    MAVCAM_LRF_MODE,
    MAVCAM_CAMERA_EFFECT,
    MAVCAM_CAMERA_SCENE,
    MAVCAM_CAMERA_AE_COMPENSATION,
    MAVCAM_CAMERA_WHITE_BALANCE,
    MAVCAM_CAMERA_AF_MODE,
    MAVCAM_CAMERA_ISO,
    MAVCAM_CAMERA_NOISE_REDUCTION,
    MAVCAM_CAMERA_SHARPNESS,
    MAVCAM_RUNNING_LOW_CAPACITY,
};


/**
 * mavlink camera status
 * notify when camera status changed
 **/
enum mavcam_status_t{
    MAVCAM_GPS_UPDATE = 0,
    MAVCAM_DRONE_ATTITUDE_UPDATE
};

/**
 * mavlink gimbal status
 * notify when gimbal status changed
 **/
enum gb_status_t{
    GIMBAL_ORIENTATION_UPDATE = 0,
    GIMBAL_MODE_UPDATE,
    GIMBAL_DEVICE_SET_ATTITUDE,
    GIMBAL_DEVICE_OUTPUT,
};

/**
 * state Capture Image
 **/
enum state_capture_image{
    STATE_CAPTURE_ERROR = -1,
    STATE_CAPTURE_IDLE  = 0,
    STATE_CAPTURE_INIT  = 1,
    STATE_CAPTURE_RUN   = 2,
};

/**
 * video record status
 **/
struct record_status_t{
    int image_capture_status;
    bool video_record_status;
    int image_capture_interval;
    int video_rec_time;

    record_status_t(){
        image_capture_status = false;
        video_record_status = false;
        image_capture_interval = 0;
        video_rec_time = 0;
    }
};

#define TRACKING_SCREEN_WIDTH_EO 640
#define TRACKING_SCREEN_HEIGHT_EO 480
#define TRACKING_BOUND_SIZE_EO 64
#define TRACKING_SEARCH_WINDOW_W_EO 640
#define TRACKING_SEARCH_WINDOW_H_EO 480
#define TRACKING_DEFAULT_BOUND_X_EO ((TRACKING_SCREEN_WIDTH_EO - TRACKING_BOUND_SIZE_EO) / 2)
#define TRACKING_DEFAULT_BOUND_Y_EO ((TRACKING_SCREEN_HEIGHT_EO - TRACKING_BOUND_SIZE_EO) / 2)

#define TRACKING_SCREEN_WIDTH_IR 1280
#define TRACKING_SCREEN_HEIGHT_IR 1024
#define TRACKING_BOUND_SIZE_IR 128
#define TRACKING_SEARCH_WINDOW_W_IR 960
#define TRACKING_SEARCH_WINDOW_H_IR 720
#define TRACKING_DEFAULT_BOUND_X_IR ((TRACKING_SCREEN_WIDTH_IR - TRACKING_BOUND_SIZE_IR) / 2)
#define TRACKING_DEFAULT_BOUND_Y_IR ((TRACKING_SCREEN_HEIGHT_IR - TRACKING_BOUND_SIZE_IR) / 2)

enum osd_object_id_t{
    OSD_STATUS_SYS_DATE = 0,
    OSD_STATUS_SYS_TIME,
    OSD_STATUS_EO_ZOOM,
    OSD_STATUS_IR_ZOOM,
    OSD_STATUS_LRF,
    OSD_STATUS_IP_ADDR,

    OSD_STATUS_TAG_CAP,
    OSD_STATUS_TAG_LON,
    OSD_STATUS_TAG_LAT,
    OSD_STATUS_TAG_ALT,

    OSD_STATUS_ACFT_CAP,
    OSD_STATUS_ACFT_LON,
    OSD_STATUS_ACFT_LAT,
    OSD_STATUS_ACFT_ALT,

    OSD_STATUS_RADIOMETRIC_CAP,
    OSD_STATUS_RADIOMETRIC_MAX,
    OSD_STATUS_RADIOMETRIC_MIN,
    OSD_STATUS_RADIOMETRIC_POINT,

    OSD_STATUS_REC_SRC,
    OSD_STATUS_REC_TIME,

    OSD_DEBUG_TRACK_CAP=0,
    OSD_DEBUG_TRACK_ROLL,
    OSD_DEBUG_TRACK_PITCH,
    OSD_DEBUG_TRACK_YAW,
    OSD_DEBUG_FPS,

    OSD_DEBUG_DRONE_CAP,
    OSD_DEBUG_DRONE_PITCH,
    OSD_DEBUG_DRONE_ROLL,
    OSD_DEBUG_DRONE_YAW,

    OSD_DEBUG_MAV_RC_CAP,
    OSD_DEBUG_MAV_RC_PITCH,
    OSD_DEBUG_MAV_RC_ROLL,
    OSD_DEBUG_MAV_RC_YAW,

    OSD_DEBUG_GIMBAL_CAP,
    OSD_DEBUG_GIMBAL_VER,
    OSD_DEBUG_GIMBAL_PITCH,
    OSD_DEBUG_GIMBAL_ROLL,
    OSD_DEBUG_GIMBAL_YAW,

    OSD_DEBUG_IP_ADDR,

    OSD_DEBUG_TEMP_CAP,
    OSD_DEBUG_TEMP_CPU,
    OSD_DEBUG_TEMP_GPU,

    OSD_DEBUG_EXT_SENSORS_CAP,
    OSD_DEBUG_EXT_SENSORS_TEMP,
    OSD_DEBUG_EXT_SENSORS_HUMI,
    OSD_DEBUG_EXT_SENSORS_PRESS,

    OSD_DEBUG_APP_VER,
    OSD_DEBUG_GB_VER,

    OSD_DEBUG_DB_CAP,
    OSD_DEBUG_DB_0,
    OSD_DEBUG_DB_1,
    OSD_DEBUG_DB_2,
    OSD_DEBUG_DB_3,
    OSD_DEBUG_DB_4,
    OSD_DEBUG_DB_5,
    OSD_DEBUG_DB_6,
    OSD_DEBUG_DB_7,
    OSD_DEBUG_DB_8
};

enum radiometric_event_t{
    RADIOMETRIC_EVENT_MAX = 0,
    RADIOMETRIC_EVENT_MIN,
    RADIOMETRIC_EVENT_POINT
};

struct zoom_info_t
{
    int zoom_mode;
    double zoom_level;
    double zoom_value;
    int zoom_status;
    zoom_info_t(){
        zoom_level = 0;
        zoom_mode = 0;
        zoom_value = 0;
        zoom_status = 0;
    }
};
   
struct remote_ctrl_info_t
{
    double pitch;
    double roll;
    double yaw;
    remote_ctrl_info_t(){
        pitch = 0;
        roll = 0;
        yaw = 0;
    }
};
 
static std::string getSystemDate(){
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%d/%m/%Y", &tstruct);
    
    return buf;
}

static std::string getSystemTime(){
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];

    tstruct = *gmtime(&now);
    sprintf(buf,"%02d:%02d:%02d",((tstruct.tm_hour+ICT)%24), tstruct.tm_min, tstruct.tm_sec);

    return buf;
}

static std::string get_local_time(){
    std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time( std::localtime( &t ), "%FT%T%z" );
    return ss.str();
}

static void setLocalTimeFromEpoch(uint64_t epoch){
    int32_t ret;
    char cmdStr[128];

    // If epoch is 0 (wrong timestamp from GPS), return to avoid error logs.
    if (epoch == 0ULL)
        return;

    memset(cmdStr, 0, sizeof(cmdStr));
#if defined QC_PLATFORM || defined OCLEA_PLATFORM || defined GENIO_PLATFORM
    snprintf(cmdStr, sizeof(cmdStr), "date -s '@%ld' > /dev/null", epoch);
#else
    snprintf(cmdStr, sizeof(cmdStr), "sudo date -s '@%ld' > /dev/null", epoch);
#endif /* QC_PLATFORM */

    ret = system(cmdStr);

    // printf("%s %d \n", __func__, ret);
}


#endif