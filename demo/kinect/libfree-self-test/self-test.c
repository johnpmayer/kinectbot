#include <stdlib.h>
#include <stdio.h>
#include <libfreenect_sync.h>
#include <inttypes.h>

int main()
{
  
  uint16_t* data;

  uint32_t timestamp;

  printf("Start\n");  
  
  uint64_t movAvg[3] = {0,0,0};
  
  while(1) {
    
    freenect_sync_get_depth( (void**)(&data), 
			     &timestamp, 
			     0, 
			     FREENECT_DEPTH_11BIT );
    
    printf("Got data form depth camera\n");
    
    uint32_t offset;
    uint32_t max = 640*488;
    uint64_t sum = 0;
    
    for(offset = 0; offset < max; offset ++ ){
      
      //printf("%" PRIx32 ":\t%" PRIu16 "\n", offset, data[offset]);
      
      sum += data[offset];
      
    }
    
    movAvg[0] = movAvg[1];
    movAvg[1] = movAvg[2];
    movAvg[2] = sum / max;
    
    printf("Avg: %" PRIu64 "\n", 
	   (movAvg[0]+movAvg[1]+movAvg[2])/3
	    );
    
  }

  return 0;
  
}
