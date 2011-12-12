
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

#define THRESH 150

// Globals

char command;

double posX;
double posY;
double posT;

pthread_mutex_t _lock;
//pthread_mutex_t* lock = &_lock;

//Roomba* roomba;

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

void *exc_cmd(void* _roomba)
{
  
  while(1)
    {
      int err = 0;
      //      err = sem_wait(sem_exc_cmd);
      
      printf("exc lock \n");
      pthread_mutex_lock(&_lock);
      
      if (err != 0) 
	{
	  printf("err: %d\n",err);
	  perror("exc sem wait");
	  exit(1);
	}
      
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
	  roomba_spinleft(_roomba);
	  break;

	case'd':
	  roomba_spinright(_roomba);
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
	  
	  double dist = dist_i;
	  double angle = angle_i / 360.0 * 2.0 * PI;
	  
	  posT += angle;
	  
	  posX += dist * cos(posT);
	  posY += dist * sin(posT);
	  
	  /*printf("delta dist: %f"  " delta angle: %f"  "\n",
	    dist,angle);*/
	  
	  break;
	}
      
      //command = 'q';
      
      //      sem_post(sem_ready_for_cmd);
      printf("exc unlock\n");
      pthread_mutex_unlock(&_lock);
      
    }

  pthread_exit(NULL);

}


int main(int argc, char* argv[])
{
  printf("yes\n");
  
  init();
  
  Roomba* roomba = roomba_init( argv[1] );
  
  pthread_mutex_init(&_lock, NULL);
  
  pthread_t texc_cmd;
  pthread_create(&texc_cmd, NULL, exc_cmd, (void*)roomba);
  
  printf("Threads going\n");
  
  while(1)
    {
      
      //     printf("Getting obstacles\n");
      
      uint8_t* obs = findObstacles();
      
      uint8_t* cols = calloc ( (W / REGION_RES),
			       sizeof(uint8_t));
      
      //printf("Walking obstacles\n");
      
      int i,j;
      for (i = 0; i < (H / REGION_RES); i++){
	for (j = 0; j < (W / REGION_RES); j++){
	  
	  //printf("checking obstacle %d %d\n", i, j);
	  
	  uint8_t region_avg = obs[i * (W / REGION_RES) + j];
	  
	  if (region_avg > THRESH) {
	    cols[j] = 1;
	  }
	  
	  //	  printf("%d ", region_avg);
	}
	//	printf("\n");
      }
      
      //printf("Finished with raw obstacles");
      
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

      printf("main lock\n");
      pthread_mutex_lock(&_lock);
      command = tempcmd;
      printf("main unlock\n");
      pthread_mutex_unlock(&_lock);
      
    }
  
  pthread_join(texc_cmd, NULL);
  
  return 0;
  
}
