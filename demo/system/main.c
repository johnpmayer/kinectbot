

#include "../roomba/simpletest.c"
#include "../kinect/regions/depth_regions.c"
//#include "../roomba/roombalib.c"

uint8_t* obstacleGrid;

#define N_COLS (W / REGION_RES)
#define N_ROWS (H / REGION_RES)

int main() {
  
  printf("yes links!\n");
  
  printf("%d %d\n", N_COLS, N_ROWS);
  
  while(1)
    {
      
      obstacleGrid = getRegionObstacles();
      
      uint8_t obstacle = 0;
      
      uint8_t columns[N_COLS];

      for (int i = 0; i < N_COLS; i++)
	{
	  columns[i] = 0;
	}
      
      for (int i = 0; i < N_COLS*N_ROWS; i++)
	{
	  //printf("%d",obstacleGrid[i]);
	  if (obstacleGrid[i])
	    {
	      columns[i % N_COLS] = 1;
	      obstacle = 1;
	    }
	}
      
      //printf("\t");
      
      for (int i = 0; i < N_COLS; i++)
	{
	  printf("%d", columns[i]);
	}
      
      printf("\t");
      
      if (obstacle)
	{
	  int sum = 0, count = 0;
	  for (int i = 0; i < N_COLS; i++) {
	    if(columns[i]) 
	      {
		sum += i; count++;
	      }
	  }
	  double weight = sum/count;
	  //printf("weight: %d %d %f\t", sum, count, (double)(sum/count));
	  printf("warning, ");

	  if (weight > 3.5) 
	    {
	      printf("obstacle RIGHT");
	    }
	  else
	    {
	      printf("obstacle LEFT");
	    }
	  printf("\n");
	}
      else
	{
	  printf("Proceed...\n");
	}
      
      free(obstacleGrid);
      
    }

  return 0;
  
}
