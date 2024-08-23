#ifndef TRACKING_PARAMETERS_H_
#define TRACKING_PARAMETERS_H_

enum Tracking_State
{
	trk_idle = 0,
	trk_init,
	trk_zoom_in,
	trk_update_roi,
	trk_update_tracking,
	trk_init_gimbal,
	trk_move_gimbal,
	trk_lost_object,
	trk_user_stop,
	trk_delay
};

enum Tracking_Gimbal_State{
	gb_wait_heartbeat = 0,
	gb_do_setup,
	gb_move_by_tracking,
	gb_move_by_remote,
	gb_do_testing
};

struct Tracking_Result{
	int fps;
	int bbox_x;
	int bbox_y;
	double delta_x;
	double delta_y;
	bool status;
	float confidence;

	Tracking_Result(){
		reset();
	}

	void reset(){
		fps = 0;
		bbox_x = 0;
		bbox_y = 0;
		delta_x = 0.0;
		delta_y = 0.0;
		status = false;
		confidence = 0.0;
	}
};

struct Tracking_Center{
	int center_x;
	int center_y;

	Tracking_Center(){
		reset();
	}
	void reset(){
		center_x = 0;
		center_y = 0;
	}
};

struct Tracking_Config{

	int tracking_area_w;
	int tracking_area_h;
	int tracking_bbox_h;
	int tracking_bbox_w;
	int search_window_h;
	int search_window_w;
	float window_scale_h;				// we scaled realtime frame to smaller for increase fps, h= 1080 / 480
	float window_scale_w;				// we scaled realtime frame to smaller for increase fps, h= 1920 / 640
	int gimbal_windowL;					// window for gimbal moving
	int gimbal_windowH;					// window for gimbal moving
	int center_x;
	int center_y;

	Tracking_Config(){
		opentld_config();
	}

	void opentld_config(){
		tracking_area_w = 1920;
		tracking_area_h = 1080;
		// tracking_bbox_h = 64;
		// tracking_bbox_w = 64;
		tracking_bbox_w = 128;
		tracking_bbox_h = 128;

		// search_window_h = 720;		// 480 default
		// search_window_w = 1280;		// 640 default

		search_window_w = 960;			// 640 default
		search_window_h = 720;			// 480 default

		window_scale_h = 1;				// h= 1080 / 480
		window_scale_w = 1;				// w= 1920 / 640
		gimbal_windowL = 0;
		gimbal_windowH = 10;
		center_x = 0;
		center_y = 0;
	}

	void default_config(){
		tracking_area_w = 640;
		tracking_area_h = 480;
		tracking_bbox_h = 64;
		tracking_bbox_w = 64;

		search_window_h = 1080 / 2;
		search_window_w = 1920 / 2;

		window_scale_h = 2.25;			// h= 1080 / 480
		window_scale_w = 3;				// w= 1920 / 640
		gimbal_windowL = 5;
		gimbal_windowH = 10;
		center_x = 0;
		center_y = 0;
	}

	void test(){
		tracking_area_w = 1920;
		tracking_area_h = 1080;
		tracking_bbox_h = 128;
		tracking_bbox_w = 128;

		search_window_h = 1080 / 2;
		search_window_w = 1920 / 2;

		window_scale_h = 1;				// h= 1080 / 480
		window_scale_w = 1;				// w= 1920 / 640
		gimbal_windowL = 5;
		gimbal_windowH = 10;
	}

	void surf_config(){
		tracking_area_h = 720;
		tracking_area_w = 1280;
		tracking_bbox_h = 200;
		tracking_bbox_w = 200;

		search_window_h = 1080 / 2;
		search_window_w = 1920 / 2;

		window_scale_h = 1.5;			// h= 720 / 480
		window_scale_w = 1.5;			// w= 1280 / 640
		gimbal_windowL = 5;
		gimbal_windowH = 10;
	}

};

#endif