/* Constants and Macros */
#define PI 3.1415926
#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))

// Kinect View region constants
#define W 640
#define H 480
#define REGION_RES 40

// Is obstacle distance metric
#define D_THRESH 150

// for detecting non-obstacled red things
#define R_COUNT_THRESH 75
#define HSV_CNT_THRESH_CLOSE 300 

// for detecting 'redness' of obstacle masks (failsafe)
#define RED_RATIO_THRESH .5
#define HSV_CNT_THRESH_FAR 100 

// Modes of operation
#define MODE_SEEK 0
#define MODE_UTURN 1
#define MODE_RETURN 2
#define MODE_FINISH 3

// Angle precision
#define UTURN_THRESH 0.1

// Avoidance 'strafe' constants
#define Y_AVOID_S 350
#define Y_AVOID_L 350

#define CENTER_COLUMN_R 150
