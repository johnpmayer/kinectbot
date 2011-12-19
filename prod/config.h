/* Constants and Macros */
#define PI 3.1415926
#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))
#define W 640
#define H 480
#define REGION_RES 40
#define D_THRESH 150
#define R_COUNT_THRESH 75
#define HSV_CNT_THRESH_CLOSE 150
#define HSV_CNT_THRESH_FAR 100
#define MODE_SEEK 0
#define MODE_UTURN 1
#define MODE_RETURN 2
#define MODE_FINISH 3
#define UTURN_THRESH 0.1
#define Y_AVOID_S 200
#define Y_AVOID_L 400
#define RED_RATIO_THRESH .5
