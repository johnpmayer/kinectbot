
#include <string.h>

#include "rgb_to_hsv_int.c"

double depthToMillimeters(uint16_t raw_depth) {
  if (GET11(raw_depth) < 2047)
    {
      return 1000.0 / (GET11(raw_depth) * -0.0030711016 + 3.3309495161);
    }
  return 0;
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

double redObstacleRatio(uint8_t* obstacles, int hsv_count_thresh)
{
  
  uint8_t* data;
  
  uint32_t* regions = calloc( (W / REGION_RES) * (H / REGION_RES), 
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
  
  uint32_t offset;
  uint32_t max = W*H;
  
  for(offset = 0; offset < max; offset += 3 )
    {
      
      uint8_t r = data[offset];
      uint8_t g = data[offset+1];
      uint8_t b = data[offset+2];
      
      struct rgb_color rgb = {r,g,b};
      struct hsv_color hsv = rgb_to_hsv(rgb);
      
      if ( (hsv.val > 32) &&
	   (hsv.hue < 24 || hsv.hue > 232) &&
	   (hsv.sat > 128)
	   )
	{
	  regions[get_region(offset)] ++;
	}
      
    }
  
  uint32_t obs_count = 0, red_count = 0;
  int i,j;
  for (i = 0; i < (H/REGION_RES); i++) 
    {
      for (j = 0; j < (W/REGION_RES); j++) 
	{
	  int region_index = i * (W/REGION_RES) + j;
	  
	  if (obstacles[region_index])
	    {
	      obs_count++;
	      if (regions[region_index] > hsv_count_thresh)
		{
		  red_count++;
		}
	    }
	}
    }
  
  free(regions);
  
  return (double)red_count / (double)obs_count;
  
}

int getRedCount() {
  
  uint32_t size = (W / REGION_RES) * (H / REGION_RES);
  
  uint8_t* all_obs = calloc( size, sizeof(uint8_t) );

  memset(all_obs, 1, size);
  
  double ratio = redObstacleRatio(all_obs, HSV_CNT_THRESH_CLOSE);
  
  free(all_obs);
  
  return (int)(ratio * size);
  
  /*
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
      
      
       * dot prod
       *
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
  */
}

double getClosestPoint() {
  
  uint16_t* data;
  uint32_t timestamp;
  int err = freenect_sync_get_depth( (void**)(&data),
				     &timestamp,
				     0,
				     FREENECT_DEPTH_11BIT );
  
  if (err)
    {
      printf("bad kinect access\n");
      exit(1);
    }
  
  double minDistance = 0.0;
  
  int i,j;
  for (i = 0; i < H; i++)
    {
      for (j = 0; j < W; j++)
	{
	  int offset = i*W + j;
	  double distance_mm = depthToMillimeters(data[offset]);
	  if (distance_mm != 0 && (minDistance == 0 || distance_mm < minDistance))
	    {
	      minDistance = distance_mm;
	    }
	}
    }
  
  return minDistance;
  
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
