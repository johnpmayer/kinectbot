
#include <unistd.h>
#include <stdint.h>

uint8_t* obstacleGrid;

#define N_COLS (W / REGION_RES)
#define N_ROWS (H / REGION_RES)

int main(int argc, char* argv[]) {
  
  //printf("yes links!\n");
  
  //printf("%d %d\n", N_COLS, N_ROWS);
  
  int ret;
  Roomba* roomba = roomba_init(argv[1]);
  
  ret = 0;
  ret = pthread_mutex_init(lock, NULL);
  if (ret) 
    {
      perror("mutex init");
      exit(1);
    }
  
  ret = 0;
  pthread_t texc_cmd;
  ret = pthread_create(&texc_cmd, NULL, exc_cmd, (void*)roomba);
  if (ret)
    {
      perror("create exc task");
      exit(1);
    }
  
  while(1)
    {
      
      obstacleGrid = getRegionObstacles();
      
      uint8_t obstacle = 0;
      char direction = '-';
      
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
	      direction = 'L';
	    }
	  else
	    {
	      printf("obstacle LEFT");
	      direction = 'R';
	    }
	  printf("\n");
	}
      else
	{
	  printf("Proceed...\n");
	}
      
      free(obstacleGrid);
      
      /* process state and decide on command to send */
      
      pthread_mutex_lock(lock);
      
      double x,y,theta;
      
      x = pos.x;
      y = pos.y;
      theta = pos.theta;
      
      if (!obstacle) 
	{
	  cmd.command = 'w';
	} 
      else 
	{
	  if (direction == 'R')
	    {
	      cmd.command = 'd';
	    }
	  else
	    {
	      cmd.command = 'a';
	    }
	}
      
      pthread_mutex_unlock(lock);
      
    }
  
  pthread_join(texc_cmd, NULL);
  roomba_close( roomba );
  roomba_free( roomba );
  
  return 0;
  
}
