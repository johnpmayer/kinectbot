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

#define REGION_RES 80

// some value between 0 and 255
#define THRESH 200

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
      
      uint16_t* data;
      
      uint64_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), 
				  sizeof(uint64_t) );
      
      uint32_t timestamp;
      
      int32_t err = freenect_sync_get_depth( (void**)(&data), 
					     &timestamp, 
					     0, 
					     FREENECT_DEPTH_11BIT );
      
      if (err) {
	delwin(win);
	endwin();
	refresh();
	
	printf("can't access kinect\n");
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
      
      
      int obstacle = 0;
      int i,j;
      int base_row = 7;
      int base_col = 5;
      for (i = 0; i < (H / REGION_RES); i++) {
	
	mvaddstr(base_row + 3 * i, base_col, "|");
	
	for (j = 0; j < (W / REGION_RES); j++) {
	  
	  uint64_t region_avg = 
	    regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
	  
	  char buf[16];
	  
	  if (region_avg > THRESH) 
	    {
	      obstacle = 1;
	      sprintf(buf, "(%" PRIu64 ")    ", region_avg);
	    }
	  else
	    {
	      sprintf(buf, "%" PRIu64 "    ", region_avg);
	    }
	  
	  
	  mvaddstr( base_row + 3 * i,
		    base_col + 6 * (j+1),
		    buf );
	  
	  
	  
	}
	
	mvaddstr( base_row + 3 * i, 
		  base_col + 6 * ((W/REGION_RES)+1), 
		  "|");
	
      }
      
      if(obstacle) {
	mvaddstr(5,3,"WARNING!\n");
      } else {
	mvaddstr(5,3,"Safe....\n");
      }
      
      refresh();
      
      free(regions);
      
    } while (LOOP);
    
  delwin(win);
  endwin();
  refresh();
  return exit_code;
  
}
