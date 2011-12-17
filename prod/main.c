
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <libfreenect_sync.h>

#include "roombalib.h"

#define PI 3.1415926

#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))

#define W 640
#define H 480

#define REGION_RES 40

#define D_THRESH 200
#define R_THRESH .26
#define R_COUNT_THRESH 100

#define MODE_SEEK 0
#define MODE_UTURN 1
#define MODE_RETURN 2
#define MODE_FINISH 3

#define UTURN_THRESH 0.1

// Globals

double posX = 0.0;
double posY = 0.0;
double posT = 0.0;

double normalize_angle(double in)
{
  double out = in;
  
  while(out > PI) {
    out -= 2*PI;
    printf("normalized posT down\n");
  }
  
  while(out < -PI) {
    out += 2*PI;
    printf("normalized posT up\n");
  }
  
  return out;
}

double angle_diff(double target, double test) 
{
  
  return normalize_angle(test - target);
  
}

uint32_t get_region(uint32_t offset)
{
  
  uint32_t total_region_cols = W / REGION_RES;
  
  uint32_t col = offset / REGION_RES;
  uint32_t reg_col = col % total_region_cols;
  
  uint32_t row = offset / W;
  uint32_t reg_row = row / REGION_RES;
  
  uint32_t region_index = (reg_row * total_region_cols) + reg_col;
  return region_index;
  
}

uint16_t t_gamma[2048];

void init(void) {
  int i;
  for (i=0; i<2048; i++) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }
}

int getRedCount() {
  uint8_t* data;
      
  double* regions = calloc( (W / REGION_RES) * (H / REGION_RES), 
			    sizeof(double) );
  
  uint32_t timestamp;
  
  int32_t err = freenect_sync_get_video( (void**)(&data), 
					 &timestamp, 
					 0, 
					 FREENECT_VIDEO_RGB );
  
  if (err) {
    
    printf("can't access kinect\n");
    exit(1);
  }

  // pull distance, calc dot product, etc
  uint32_t offset;
  uint32_t max = W*H;
  
  for(offset = 0; offset < max; offset += 3 )
    {
      
      double r = data[offset];
      double g = data[offset+1];
      double b = data[offset+2];
      
      /*
       * dot prod
       */
      double cosT = 
	r /
	sqrt( pow(r,2) + pow(g,2) + pow(b,2) );
      
      regions[get_region(offset)] += cosT;
      
    }
  
  int red_regions = 0;
  int i,j;
  for (i = 0; i < (H / REGION_RES); i++) {
    
    for (j = 0; j < (W / REGION_RES); j++) {
      
      double region_avg = 
	regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
      
      if (region_avg > R_THRESH) 
	{
	  red_regions ++;
	}
      
    }
    
  }
  
  free(regions);
  
  return red_regions;
  
}

uint8_t* findObstacles() {
  
  uint16_t* data;
  
  uint64_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), 
			      sizeof(uint64_t) );
  
  uint8_t* obstacles = calloc( (W / REGION_RES) * (H / REGION_RES), 
			       sizeof(uint8_t) );
  
  uint32_t timestamp;
  
  int32_t err = freenect_sync_get_depth( (void**)(&data), 
					 &timestamp, 
					 0, 
					 FREENECT_DEPTH_11BIT );
  
  if (err)
    {
      printf("bad kinect access\n");
      exit(1);
    }
  
  
  uint32_t offset;
  uint32_t max = W*H;
  
  for(offset = 0; offset < max; offset ++ )
    {
      
      uint16_t depth = GET11(data[offset]);
      
      uint16_t pval = t_gamma[depth];
      int lb = pval & 0xff;
      
      /*
       * what we really want here is only the closest
       * detected objects, so we only look at case 0:
       */
      uint8_t pixel;// = picture[3*offset];
      
      if (pval>>8 == 0) {
	pixel = 255-lb;
      } else {
	pixel = 0;
      }
      
      regions[get_region(offset)] += pixel;
    }
  
  int i,j;
  for (i = 0; i < (H / REGION_RES); i++) {
    
    for (j = 0; j < (W / REGION_RES); j++) {
      
      uint64_t region_avg = 
	regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
      
      //printf("%lld ", region_avg);
      
      obstacles[(i * (W / REGION_RES)) + j] = region_avg & 0xFF;
      
    }
    
    //    printf("\n");
    
  }

  //  printf("\n");
  
  free(regions);
  
  return obstacles;
  
}
void exc_one(Roomba* _roomba, char command)
{
  
  printf("exc command: %c\n", command);
  
  switch(command)
    {
      
    case'w':
      roomba_forward(_roomba);
      break;
      
    case's':
      roomba_backward(_roomba);
      break;
      
    case'a':
      roomba_spinleft_at(_roomba,100);
      break;
      
    case'd':
      roomba_spinright_at(_roomba,100);
      break;
      
    case'p':
      roomba_stop(_roomba);
      break;
      
    case'e':
      roomba_stop(_roomba);
      pthread_exit(NULL);
      break;
      
    default:
      
      // sleep
      roomba_delay(100);
      
      roomba_read_sensors(_roomba);
      uint8_t* sb = ((Roomba*)_roomba)->sensor_bytes;
      
      int16_t dist_i  = (sb[12]<<8) | sb[13];
      int16_t angle_i = (sb[14]<<8) | sb[15]; // in degrees
      
      printf("delta dist:%d angle:%d\n", dist_i, angle_i);
      
      double dist = dist_i;
      double angle = angle_i / 360.0 * 2.0 * PI;
      
      posT += angle;
      
      posT = normalize_angle(posT);
      
      posX += dist * cos(posT);
      posY += dist * sin(posT);
      
      break;
    }
  
}

int main(int argc, char* argv[])
{
  printf("yes\n");
  
  init();
  
  Roomba* roomba = roomba_init( argv[1] );

  exc_one(roomba, 'q');
  
  posX = 0.0;
  posY = 0.0;
  posT = 0.0;

  uint8_t mode = MODE_SEEK;
  
  printf("Start loop");
  
  while(1)
    {
      
      /*
       * Get obstacles and print the status of each column
       */
      
      uint8_t* obs = findObstacles();
      uint8_t* cols = calloc ( (W / REGION_RES),
			       sizeof(uint8_t));
      
      int i,j;
      for (i = 0; i < (H / REGION_RES); i++){
	for (j = 0; j < (W / REGION_RES); j++){
	  uint8_t region_avg = obs[i * (W / REGION_RES) + j];
	  if (region_avg > D_THRESH) {
	    cols[j] = 1;
	  }
	}
      }
      
      free(obs);
      
      printf("Cols: ");
      
      uint32_t weight = 0;
      uint32_t area = 0;
      uint8_t anyobs = 0;
      
      for (i = 0; i < (W / REGION_RES); i++) {
	if (cols[i]) {
	  weight += i;
	  area++;
	  anyobs = 1;
	}
	printf("%d", cols[i]);
      }
      printf("\n");
      
      double centerofmass = (double)weight/(double)area;
      
      char tempcmd = 'q';
      
      if (anyobs) {
	if (centerofmass > (((W / REGION_RES)+1.0)/2.0) ) {
	  printf("obs right, go LEFT\n");
	  tempcmd = 'a';
	} else {
	  printf("obs left, go RIGHT\n");
	  tempcmd = 'd';
	}
      } else {
	printf("clear\n");
	tempcmd = 'w';
      }
      
      /*
       * Get the red count to decide if we can leave the SEEK mode
       */
      
      int red_count = getRedCount();
      
      printf("red count: %d\n", red_count);
      
      if (red_count > R_COUNT_THRESH)
	{
	  printf("RED OBJECT DETECTED\n");
	  tempcmd = 'p';
	  if (mode == MODE_SEEK)
	    {
	      mode = MODE_UTURN;
	    }
	  else if (mode == MODE_RETURN)
	    {
	      mode = MODE_FINISH;
	    }
	  else
	    { 
	      printf("mode error");
	    }
	}
      
      /*
       * Query the distanced traveled, which updates our position
       */
            
      exc_one(roomba, 'q');
      printf("x:%f, y:%f, t:%f\n", posX, posY, posT);
      
      switch(mode)
	{

	case MODE_SEEK:
	  //printf("fake send %c\n", tempcmd);
	  exc_one(roomba, tempcmd);      
	  break;
	  
	case MODE_RETURN:
	  exc_one(roomba, tempcmd);
	  break;
	  
	case MODE_FINISH:
	  exc_one(roomba, 'p');

	  int i;
	  for (i = 0; i < 5; i++)
	    {
	      roomba_play_note(roomba, 10, 10);
	      roomba_delay(10);
	    }
	  
	  printf("returned!\n");
	  
	  exit(0);
	  break;
	  
	case MODE_UTURN:
	  printf("ToDo: turning around\n");
	  
	  double return_bearing = atan2(-posY, -posX);
	  
	  double diff = angle_diff(return_bearing, posT);
	  double abs_diff = (diff < 0) ? -diff : diff;
	  
	  printf("return_bearing:%f posT:%f diff:%f (%f)\n", 
		 return_bearing, posT, diff, abs_diff);
	  
	  while (abs_diff > UTURN_THRESH)
	    {
	      
	      if (diff > 0) 
		{
		  exc_one(roomba, 'd');
		}
	      else
		{
		  exc_one(roomba, 'a');
		}
	      
	      exc_one(roomba, 'q');
	      
	      diff = angle_diff(return_bearing, posT);
	      abs_diff = (diff < 0) ? -diff : diff;
	      
	      printf("return_bearing:%f posT:%f diff:%f (%f)\n", 
		     return_bearing, posT, diff, abs_diff);
	      
	    }
	  
	  printf("turned around!\n");
	  
	  mode = MODE_RETURN;
	  break;
	  
	default:
	  printf("todo: mode %d", mode);
	  exit(1);
	  break;
	  
	}
      
    }
  
  return 0;
  
}
