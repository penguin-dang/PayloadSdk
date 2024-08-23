#ifndef PAYLOAD_CONFIG_TEMPLATE_H_
#define PAYLOAD_CONFIG_TEMPLATE_H_


static const std::string 	TARGET_STREAM_IP =	"192.168.12.114";
static const uint32_t 		TARGET_STREAM_PORT =	5000;

#if 0
// to flight controller
static char *fc_port = (char*)"/dev/ttyTHS1";
static int baudrate_fc = 115200;

// to gimbal controller
static char *gb_port = (char*)"/dev/ttyTHS0";
static int baudrate_gb = 115200;


#ifndef USE_SONYBLOCK
#define USE_SONYBLOCK
static char *VISCA_PORT = (char*)"/dev/ttyUSB1";
#endif

#ifndef USE_BOSON
#define USE_BOSON
static int BOSON_PORT_ID = 16;
#endif

// to LRF125
static char *lrf_port = (char*)"/dev/ttyUSB0";
static int baudrate_lrf = 115200;
#define PORT_BAUD baud115200
#define FREQ_MODE CMM10Hz

#else
// to flight controller
static char *fc_port = (char*)"/dev/ttyTHS1";
static int baudrate_fc = 115200;

// to gimbal controller
static char *gb_port = (char*)"/dev/ttyUSB1";
static int baudrate_gb = 115200;


#ifndef USE_SONYBLOCK
#define USE_SONYBLOCK
static char *VISCA_PORT = (char*)"/dev/ttyUSB0";
#endif

// #ifndef USE_BOSON
// #define USE_BOSON
// static int BOSON_PORT_ID = 16;
// #endif

// to LRF125
static char *lrf_port = (char*)"/dev/ttyTHS0";
static int baudrate_lrf = 115200;
#define PORT_BAUD baud115200
#define FREQ_MODE CMM10Hz
#endif

#endif