#include "payloadSdkInterface.h"

PayloadSdkInterface::PayloadSdkInterface(){
	printf("Starting Gremsy PayloadSdk %s\n", SDK_VERSION);
}

PayloadSdkInterface::PayloadSdkInterface(T_ConnInfo data){
	printf("Starting Gremsy PayloadSdk %s\n", SDK_VERSION);
	payload_ctrl_type = data.type;
	if(payload_ctrl_type == CONTROL_UART){
		payload_uart_port = (char*)data.device.uart.name;
		payload_uart_baud = data.device.uart.baudrate;
	}else if(payload_ctrl_type == CONTROL_UDP){
		udp_ip_target = (char*)data.device.udp.ip;
		udp_port = data.device.udp.port;
	}
	
}

PayloadSdkInterface::~PayloadSdkInterface(){

}

bool 
PayloadSdkInterface::
sdkInitConnection(){
	/* Port for connect with payload */
	if(payload_ctrl_type == CONTROL_UART){
    	port = new Serial_Port(payload_uart_port, payload_uart_baud);
	}else if(payload_ctrl_type == CONTROL_UDP)
	    port = new UDP_Port(udp_ip_target, udp_port);
	else{
    	printf("Please define your control method first. See payloadsdk.h\n");
    	return false;
	}
    /* Instantiate an gimbal interface object */
    payload_interface = new Autopilot_Interface(port, SYS_ID, COMP_ID, 2, MAVLINK_COMM_1);


    // quit port will close at terminator event
    port_quit        = port;


    /* Start the port and payload_interface */
    try{
	    port->start();
        payload_interface->start();
    }catch(...){
    	printf("Open Serial Port Error\r\n");
    	return false;
    }


    initGimbal((Serial_Port*)port);
}


void 
PayloadSdkInterface::
sdkQuit(){
	if(port_quit != nullptr){
		port_quit->stop();
	}
}

uint8_t 
PayloadSdkInterface::
getNewMewssage(mavlink_message_t& new_msg){
	if(payload_interface != nullptr){
		return payload_interface->get_nxt_message(new_msg);
	}
	return 0;
}

void 
PayloadSdkInterface::
moveGimbal(float pitch_spd, float yaw_spd){
	// send Do_MOUNT_CONTROL message

	mavlink_command_long_t msg ={0};


	msg.command = MAV_CMD_DO_MOUNT_CONTROL;
	msg.param1 = pitch_spd;
	msg.param2 = 0;
	msg.param3 = yaw_spd;
	msg.target_system = 1;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.confirmation = 0;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadCameraParam(char param_id[], uint32_t param_value, uint8_t param_type){
	mavlink_param_ext_set_t msg={0};

	strcpy((char *)msg.param_id, param_id);

	cam_param_union_t u;
    u.param_uint32 = param_value;
    std::string str(reinterpret_cast<char const *>(u.bytes), CAM_PARAM_VALUE_LEN);
	strcpy(msg.param_value, str.c_str());

	msg.param_type = param_type;
	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_param_ext_set_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraSettingList(){
	mavlink_param_ext_request_list_t msg= {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	// msg.trimmed = 0;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_param_ext_request_list_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadStorage(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_REQUEST_STORAGE_INFORMATION;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCaptureStatus(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraMode(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_REQUEST_CAMERA_SETTINGS;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraInformation(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_REQUEST_CAMERA_INFORMATION;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	message.sysid = SYS_ID;
	message.compid = COMP_ID;
	
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
getPayloadCameraStreamingInformation(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadCameraMode(CAMERA_MODE mode){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_SET_CAMERA_MODE;
	msg.param2 = (uint32_t)mode;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadCameraCaptureImage(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_IMAGE_START_CAPTURE;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
setPayloadCameraRecordVideoStart(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_VIDEO_START_CAPTURE;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void
PayloadSdkInterface::
setPayloadCameraRecordVideoStop(){
	mavlink_command_long_t msg = {0};

	msg.target_system = PAYLOAD_SYSTEM_ID;
	msg.target_component = PAYLOAD_COMPONENT_ID;
	msg.command = MAV_CMD_VIDEO_STOP_CAPTURE;
	msg.confirmation = 1;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;

	mavlink_msg_command_long_encode_chan(SYS_ID, COMP_ID, port->get_mav_channel(), &message, &msg);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	payload_interface->push_message_to_queue(message);
}

void 
PayloadSdkInterface::
initGimbal(Serial_Port* port){
	_system_id.sysid = SYS_ID;
	_system_id.compid = COMP_ID;
	myGimbalPort = port;
	myGimbal = new Gimbal_Protocol_V2(myGimbalPort, _system_id);

	_gimbal_id.sysid = 1;
	_gimbal_id.compid = MAV_COMP_ID_GIMBAL;
	myGimbal->initialize(_gimbal_id);
}

void 
PayloadSdkInterface::
setGimbalSpeed(float spd_pitch, float spd_roll, float spd_yaw, Gimbal_Protocol::input_mode_t mode){
	if(myGimbal != nullptr){
		myGimbal->set_gimbal_move_sync(spd_pitch, spd_roll, spd_yaw, mode);
	}
}

void 
PayloadSdkInterface::
setGimbalMode(Gimbal_Protocol::control_mode_t mode){
	if(myGimbal == nullptr){
		printf("%s | %d | myGimbal is nullptr\r\n",__func__,__LINE__);
		return;
	}
	myGimbal->set_gimbal_mode_sync(mode);
}

void 
PayloadSdkInterface::
setGimbalResetMode(Gimbal_Protocol::gimbal_reset_mode_t reset_mode){
	if(myGimbal == nullptr){
		printf("%s | %d | myGimbal is nullptr\r\n",__func__,__LINE__);
		return;
	}
	auto ret = myGimbal->set_gimbal_reset_mode(reset_mode);
}


void 
PayloadSdkInterface::
setGimbalPowerOn()
{
	if(myGimbal == nullptr){
		printf("%s | %d | myGimbal is nullptr\r\n",__func__,__LINE__);
		return;
	}

	const float para[7] = {
		0,									//para 1																							
		0,									//para 2
		0,									//para 3
		0,									//para 4
		0,									//para 5
		0,									//para 6
		1.0f								//para 7
	};

	auto ret = myGimbal->send_command_long(MAV_CMD_USER_1,para);
	printf("%s | return : [%s]\r\n",__func__,(ret == Gimbal_Protocol::SUCCESS) ? "SUCCESS" : "ERROR");
}


void 
PayloadSdkInterface::
setGimbalPowerOff()
{
	if(myGimbal == nullptr){
		printf("%s | %d | myGimbal is nullptr\r\n",__func__,__LINE__);
		return;
	}

	const float para[7] = {
		0,									//para 1																							
		0,									//para 2
		0,									//para 3
		0,									//para 4
		0,									//para 5
		0,									//para 6
		0									//para 7
	};
	auto ret = myGimbal->send_command_long(MAV_CMD_USER_1,para);
	printf("%s | return : [%s]\r\n",__func__,(ret == Gimbal_Protocol::SUCCESS) ? "SUCCESS" : "ERROR");
}
