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
