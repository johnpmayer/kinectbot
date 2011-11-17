#include <stdlib.h>
#include <stdio.h>
#include <libfreenect_sync.h>

int main()
{
  
  uint8_t* data;

  uint32_t timestamp;

  printf("Start\n");  

  freenect_sync_get_depth( (void**)(&data), 
			   &timestamp, 
			   0, 
			   FREENECT_DEPTH_11BIT );
  
  printf("Got data form depth camera\n");

  return 0;
  
}
