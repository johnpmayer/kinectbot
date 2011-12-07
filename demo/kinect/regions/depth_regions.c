#include <stdlib.h>
#include <stdio.h>
#include <libfreenect_sync.h>
#include <inttypes.h>
#include <math.h>

#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))

#define W 640
#define H 480

#define REGION_RES 80

// some value between 0 and 255
#define THRESH 225

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
uint8_t isInit = 0;

void init(void) {
  int i;
  for (i=0; i<2048; i++) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }
}

uint8_t* getRegionObstacles() {

  if (!isInit) init();
  
  uint16_t* data;
  char* picture = calloc(W*H*3, sizeof(uint8_t));
  
  uint64_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), sizeof(uint64_t) );
  
  uint8_t* obstacles = calloc( (W / REGION_RES) * (H / REGION_RES), sizeof(uint8_t) );
  
  uint32_t timestamp;
  
  int32_t err = freenect_sync_get_depth( (void**)(&data), 
					 &timestamp, 
					 0, 
					 FREENECT_DEPTH_11BIT );
  
  if (err) {
    printf("can't access kinect\n");
    return NULL;
  }
  
  uint32_t offset;
  uint32_t max = W*H;
  
  for(offset = 0; offset < max; offset ++ ){
    
    //uint16_t tags = GET5(data[offset]);
    uint16_t depth = GET11(data[offset]);
    
    uint16_t pval = t_gamma[depth];
    int lb = pval & 0xff;
    
    if ( (offset % REGION_RES == 0) || ((offset/W) % REGION_RES == 0) ) {
      picture[3*offset] = -1;
      picture[3*offset+1] = -1;
      picture[3*offset+2] = -1;
    } else if (depth < 2047) {
      
      switch (pval>>8) {
      case 0:
	picture[3*offset+0] = 255;
	picture[3*offset+1] = 255-lb;
	picture[3*offset+2] = 255-lb;
	break;
      case 1:
	picture[3*offset+0] = 255;
	picture[3*offset+1] = lb;
	picture[3*offset+2] = 0;
	break;
      case 2:
	picture[3*offset+0] = 255-lb;
	picture[3*offset+1] = 255;
	picture[3*offset+2] = 0;
	break;
      case 3:
	picture[3*offset+0] = 0;
	picture[3*offset+1] = 255;
	picture[3*offset+2] = lb;
	break;
      case 4:
	picture[3*offset+0] = 0;
	picture[3*offset+1] = 255-lb;
	picture[3*offset+2] = 255;
	break;
      case 5:
	picture[3*offset+0] = 0;
	picture[3*offset+1] = 0;
	picture[3*offset+2] = 255-lb;
	break;
      default:
	picture[3*offset+0] = 0;
	picture[3*offset+1] = 0;
	picture[3*offset+2] = 0;
	break;
      }
      
    }
    
    picture[3*offset+1] = picture[3*offset];
    picture[3*offset+2] = picture[3*offset];
    
    uint8_t pixel = picture[3*offset];
    regions[get_region(offset)] += pixel;
    
  }
  
  int i,j;
  
  for (i = 0; i < (H / REGION_RES); i++) 
    {
      for (j = 0; j < (W / REGION_RES); j++) 
	{
	  
	  uint64_t region_avg = 
	    regions[(i * (W / REGION_RES)) + j] 
	    / pow(REGION_RES, 2);
	  
	  if (region_avg > THRESH)
	    {
	      obstacles[(i * (W / REGION_RES)) + j] = 1;
	    }
	  
	}
    }
  
  free(picture);
  free(regions);
  
  return obstacles;
  
}
