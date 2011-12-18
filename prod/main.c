
/* Standard Libraries */
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

/* Device Libraries */
#include <libfreenect_sync.h>
#include "roombalib.h"

#include "config.h"

/* Module Code */
#include "vision.c"

/* Globals */
double posX = 0.0;
double posY = 0.0;
double posT = 0.0;

/* Enforces that a double is in the range (-PI, PI] */
double normalize_angle(double in)
{
  double out = in;
  
  while(out > PI) {
    out -= 2*PI;
    printf("normalized posT down\n");
  }
  
  while(out <= -PI) {
    out += 2*PI;
    printf("normalized posT up\n");
  }
  
  return out;
}

/* Difference of two angles */
double angle_diff(double target, double test) 
{
  
  return normalize_angle(test - target);
  
}


/* Translate a character command code and send to roomba */
void exc_one(Roomba* _roomba, char command)
{
  
  printf("exc command: %c\n", command);
  
  switch(command)
    {
      
    case'w':
      roomba_forward_at(_roomba,100);
      break;
      
    case's':
      roomba_backward(_roomba);
      break;
      
    case'a':
      roomba_spinleft_at(_roomba,100);
      break;
      
    case'd':
      roomba_spinright_at(_roomba,100);
      break;
      
    case'p':
      roomba_stop(_roomba);
      break;
      
    case'e':
      roomba_stop(_roomba);
      pthread_exit(NULL);
      break;
      
    default:
      
      // sleep
      roomba_delay(100);
      
      roomba_read_sensors(_roomba);
      uint8_t* sb = ((Roomba*)_roomba)->sensor_bytes;
      
      int16_t dist_i  = (sb[12]<<8) | sb[13];
      int16_t angle_i = (sb[14]<<8) | sb[15]; // in degrees
      
      printf("delta dist:%d angle:%d\n", dist_i, angle_i);
      
      double dist = dist_i;
      double angle = angle_i / 360.0 * 2.0 * PI;
      
      posT += angle;
      
      posT = normalize_angle(posT);
      
      posX += dist * cos(posT);
      posY += dist * sin(posT);
      
      break;
    }
  
}

void orientToAngle(Roomba* roomba, double targetAngle)
{
  
  printf("turning to desired angle %f\n", targetAngle);
  
  double diff = angle_diff(targetAngle, posT);
  double abs_diff = (diff < 0) ? -diff : diff;
  
  printf("target angle:%f posT:%f diff:%f (%f)\n", 
	 targetAngle, posT, diff, abs_diff);
  
  while (abs_diff > UTURN_THRESH)
    {
      
      if (diff > 0) 
	{
	  //exc_one(roomba, 'd');
	  roomba_spinright_at(roomba, 50);
	}
      else
	{
	  //exc_one(roomba, 'a');
	  roomba_spinleft_at(roomba, 50);
	}
      
      exc_one(roomba, 'q');
      
      diff = angle_diff(targetAngle, posT);
      abs_diff = (diff < 0) ? -diff : diff;
      
      printf("targetAngle:%f posT:%f diff:%f (%f)\n", 
	     targetAngle, posT, diff, abs_diff);
      
    }
  
  printf("finished turning\n");
  
}

void moveMillimetersY(Roomba* roomba, double yDist) {
  
  if (yDist == 0) return;

  if (yDist > 0)
    {
      orientToAngle(roomba, PI/2);
    }
  else
    {
      orientToAngle(roomba, -PI/2);
    }
  
  double targetYPos = posY + yDist;
  
  while ( ( yDist > 0 && posY < targetYPos ) ||
	  ( yDist < 0 && posY > targetYPos ) )
    {
      roomba_forward_at(roomba, 100);
      exc_one(roomba, 'q');
      printf("targetY: %f posY: %f\n", targetYPos, posY);
    }
  
  printf("moved in y direction\n");
  
}

void moveMillimetersX(Roomba* roomba, double xDist) {
  
  if (xDist == 0) return;

  if (xDist > 0)
    {
      orientToAngle(roomba, 0);
    }
  else
    {
      orientToAngle(roomba, -PI);
    }
  
  double targetXPos = posX + xDist;
  
  while ( ( xDist > 0 && posX < targetXPos ) ||
	  ( xDist < 0 && posX > targetXPos ) )
    {
      roomba_forward_at(roomba, 100);
      roomba_delay(10);
      exc_one(roomba, 'q');
      printf("targetX: %f posX: %f\n", targetXPos, posX);
    }
  
  printf("moved in x direction\n");
  
}

/* Main */
int main(int argc, char* argv[])
{
  printf("yes\n");
  
  init();
  
  Roomba* roomba = roomba_init( argv[1] );

  exc_one(roomba, 'q');
  
  posX = 0.0;
  posY = 0.0;
  posT = 0.0;

  uint8_t mode = MODE_SEEK;
  
  printf("Start loop");
  
  while(1)
    {
      
      /*
       * Get obstacles and print the status of each column
       */
      
      double closest = getClosestPoint();
      printf("closest point %f\n", closest);
      
      uint8_t* obs = findObstacles();
      uint8_t* cols = calloc ( (W / REGION_RES),
			       sizeof(uint8_t));
      
      int i,j;
      for (i = 0; i < (H / REGION_RES); i++){
	for (j = 0; j < (W / REGION_RES); j++){
	  uint8_t region_avg = obs[i * (W / REGION_RES) + j];
	  if (region_avg > D_THRESH) {
	    cols[j] = 1;
	  }
	}
      }
      
      free(obs);
      
      printf("Cols: ");
      
      uint32_t weight = 0;
      uint32_t area = 0;
      uint8_t anyobs = 0;
      
      for (i = 0; i < (W / REGION_RES); i++) {
	if (cols[i]) {
	  weight += i;
	  area++;
	  anyobs = 1;
	}
	printf("%d", cols[i]);
      }
      printf("\n");
      
      double centerofmass = (double)weight/(double)area;
      
      /*
       * At this point we know what's in front of us and what columns are clear (sans blind spot)
       */
      
      /*
       * Get the red count to decide if we can leave the SEEK mode
       */
      
      int red_count = getRedCount();
      
      printf("red count: %d\n", red_count);
      
      if (red_count > R_COUNT_THRESH)
	{
	  printf("RED OBJECT DETECTED\n");
	  //tempcmd = 'p';
	  if (mode == MODE_SEEK)
	    {
	      mode = MODE_UTURN;
	    }
	  else if (mode == MODE_RETURN)
	    {
	      mode = MODE_FINISH;
	    }
	  else
	    { 
	      printf("mode error");
	    }
	}
      

      
      
      /*
       * Query the distanced traveled, which updates our position
       */
      
      exc_one(roomba, 'q');
      printf("x:%f, y:%f, t:%f\n", posX, posY, posT);
      
      
      
      switch(mode)
	{

	case MODE_SEEK:
	  if (anyobs) {
	    
	    moveMillimetersY(roomba, Y_AVOID);
	    orientToAngle(roomba, 0);
	    /*
	      
	      if (centerofmass > (((W / REGION_RES)+1.0)/2.0) ) {
	      printf("obs right, go LEFT\n");
	      tempcmd = 'a';
	      } else {
	      printf("obs left, go RIGHT\n");
	      tempcmd = 'd';
	      }
	    */
	    
	  } else {
	    printf("clear\n");
	    exc_one(roomba, 'w');
	  }

	  break;
	  
	case MODE_RETURN:
	  printf("todo");
	  exc_one(roomba, 'p');
	  exit(0);
	  break;
	  
	case MODE_FINISH:
	  exc_one(roomba, 'p');
	  printf("returned!\n");
	  exit(0);
	  break;
	  
	case MODE_UTURN:
	  printf("ToDo: turning around\n");
	  double return_bearing = atan2(-posY, -posX);
	  orientToAngle(roomba, return_bearing);
	  printf("turned around!\n");
	  mode = MODE_RETURN;
	  break;
	  
	default:
	  printf("todo: mode %d", mode);
	  exit(1);
	  break;
	  
	}
      
    }
  
  return 0;
  
}
