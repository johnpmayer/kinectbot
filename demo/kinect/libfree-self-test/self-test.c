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

int main()
{
  
  uint16_t* data;
  char* picture = calloc(W*H*3, sizeof(uint8_t));
  
  uint64_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), sizeof(uint64_t) );
  
  uint32_t timestamp;
  
  printf("Start\n");  
  
  //uint64_t movAvg[3] = {0,0,0};
  
  do {
    
    int32_t err = freenect_sync_get_depth( (void**)(&data), 
			     &timestamp, 
			     0, 
			     FREENECT_DEPTH_11BIT );
    
    printf("Get depth return code: %" PRIi32 "\n", err);
    
    if (err) {
      return err;
    }
    
    printf("Got data form depth camera\n");
    
    uint32_t offset;
    uint32_t max = W*H;
    //uint64_t sum = 0;
    
    for(offset = 0; offset < max; offset ++ ){
      
      //uint16_t tags = GET5(data[offset]);
      uint16_t depth = GET11(data[offset]);
      
      uint8_t pixel = depth >> 3;
      
      if ( (offset % REGION_RES == 0) || ((offset/W) % REGION_RES == 0) ) {
	picture[3*offset] = -1;
	picture[3*offset+1] = -1;
	picture[3*offset+2] = -1;
      } else if (depth < 2047) {
	picture[3*offset] = pixel;
	picture[3*offset+1] = pixel;
	picture[3*offset+2] = pixel;
      }
      
      regions[get_region(offset)] += depth;
      
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
    
  } while (0);
  
  printf("Regions:\n");
  int i,j;
  for (i = 0; i < (H / REGION_RES); i++) {
    for (j = 0; j < (W / REGION_RES); j++) {
      
      uint64_t region_avg = regions[(i * (W / REGION_RES)) + j] / pow(REGION_RES, 2);
      
      printf("%" PRIu64 "\t", region_avg);
      
    }
    printf("\n");
  }
  
  write_bmp("output.bmp", W, H, picture);
  
  return 0;
  
}
