#include <stdlib.h>
#include <stdio.h>
#include <libfreenect_sync.h>
#include <inttypes.h>
#include <math.h>

#include "bmp.c"

#define GET5(x) (x >> 11)// & ((1 << 5) - 1))
#define GET11(x) (x & ((1<<11)-1))

#define W 640
#define H 480

#define REGION_RES 80

// some value between 0 and 255
#define THRESH 225

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
  
  init();
  
  
  
  printf("Start\n");  
  
  //uint64_t movAvg[3] = {0,0,0};
  
  do {
    
    uint16_t* data;
    char* picture = calloc(W*H*3, sizeof(uint8_t));
    
    uint64_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), sizeof(uint64_t) );
    
    uint32_t timestamp;
    
    int32_t err = freenect_sync_get_depth( (void**)(&data), 
			     &timestamp, 
			     0, 
			     FREENECT_DEPTH_11BIT );
    
    //printf("Get depth return code: %" PRIi32 "\n", err);
    
    if (err) {
      printf("can't access kinect\n");
      return err;
    }
    
    //printf("Got data form depth camera\n");
    
    uint32_t offset;
    uint32_t max = W*H;
    //uint64_t sum = 0;
    
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
	
#ifdef COLOR
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
#else
	
	picture[3*offset] = pval >> 3;
	picture[3*offset+1] = pval >> 3;
	picture[3*offset+2] = pval >> 3;
	
#endif
	
      }
      
#ifdef COLOR
      /* grep red-scale */
      picture[3*offset+1] = picture[3*offset];
      picture[3*offset+2] = picture[3*offset];
#endif
      
      uint8_t pixel = picture[3*offset];
      regions[get_region(offset)] += pixel;
      //picture[3*offset];
      
      //printf("%" PRIx32 ":\t%" PRIu16 "\n", offset, data[offset]);
      
      //sum += data[offset];
      
      //sum += depth;
      
    }
    
    /*
    movAvg[0] = movAvg[1];
    movAvg[1] = movAvg[2];
    movAvg[2] = sum / max;
    
    printf("Avg: %" PRIu64 "\n", 
	   (movAvg[0]+movAvg[1]+movAvg[2])/3
	    );
    */
    
    //printf("Regions:\n");
    int obstacle = 0;
    int i,j;
    for (i = 0; !obstacle && i < (H / REGION_RES); i++) {
      for (j = 0; !obstacle && j < (W / REGION_RES); j++) {
	
	uint64_t region_avg = regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
	if (region_avg > THRESH){
	  obstacle = 1;
	}
	//printf("%" PRIu64 "\t", region_avg);
	
      }
      //printf("\n");
    }
    
    if(obstacle) {
      printf("WARNING!\n");
    } else {
      printf("Safe....\n");
    }
    
    free(picture);
    free(regions);
    
  } while (LOOP);
  
  
  
  
  
  //write_bmp("output.bmp", W, H, picture);
  
  return 0;
  
}
