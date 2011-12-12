#include <stdlib.h>
#include <stdio.h>
#include <libfreenect_sync.h>
#include <inttypes.h>
#include <math.h>
#include <ncurses.h>

#include "bmp.c"

#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))

#define W 640
#define H 480

#define REGION_RES 40

// some value between 0 and 1, comparing with some value cos(t)
#define R_THRESH 0.26

#define LOOP 1

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

int main()
{

  printf("Start");
  
  int exit_code = 0;
  
  init();
  WINDOW* win;

  if ((win= initscr()) == NULL) {
    printf("can't init ncurses");
    return 1;
  }
  
  mvaddstr(3,3,"Start\n");
  refresh();
  
  do 
    {
      
      uint8_t* data;
      
      double* regions = calloc( (W / REGION_RES) * (H / REGION_RES), 
				  sizeof(double) );
      
      uint32_t timestamp;
      
      int32_t err = freenect_sync_get_video( (void**)(&data), 
					     &timestamp, 
					     0, 
					     FREENECT_VIDEO_RGB );
      
      if (err) {
	delwin(win);
	endwin();
	refresh();
	
	printf("can't access kinect\n");
	exit(1);
      }
      
      uint32_t offset;
      uint32_t max = W*H;
      
      for(offset = 0; offset < max; offset += 3 )
	{
	  
	  double r = data[offset];
	  double g = data[offset+1];
	  double b = data[offset+2];
	  
	  /*
	   * euclidean distance from 'perfect red'
	   */
	  /*
	    uint16_t distance = sqrt( pow((255 - r),2) +
	    pow(g,2) +
	    pow(b,2) );
	  */
	  double cosT = 
	    r /
	    sqrt( pow(r,2) + pow(g,2) + pow(b,2) );
	  
	  regions[get_region(offset)] += cosT;
	  
	}
      
      
      //int obstacle = 0;
      int red_regions = 0;
      int i,j;
      int base_row = 7;
      int base_col = 5;
      for (i = 0; i < (H / REGION_RES); i++) {
	
	mvaddstr(base_row + 3 * i, base_col, "|");
	
	for (j = 0; j < (W / REGION_RES); j++) {
	  
	  double region_avg = 
	    regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
	  
	  char buf[16];
	  
	  if (region_avg > R_THRESH) 
	    {
	      //obstacle = 1;
	      red_regions ++;
	      sprintf(buf, "(%.2f)    ", region_avg);
	    }
	  else
	    {
	      sprintf(buf, "%.2f    ", region_avg);
	    }
	  
	  
	  mvaddstr( base_row + 3 * i,
		    base_col + 6 * (j+1),
		    buf );
	  
	  
	  
	}
	
	mvaddstr( base_row + 3 * i, 
		  base_col + 6 * ((W/REGION_RES)+1), 
		  "|");
	
      }
      
      char buf[8];

      sprintf(buf, "%d    ", red_regions);
      mvaddstr(5,3,buf);
      /*
      if(obstacle) {
	mvaddstr(5,3,"WARNING!\n");
      } else {
	mvaddstr(5,3,"Safe....\n");
      }
      */
      
      refresh();
      
      free(regions);
      
    } while (LOOP);
  
  delwin(win);
  endwin();
  refresh();
  return exit_code;
  
}
