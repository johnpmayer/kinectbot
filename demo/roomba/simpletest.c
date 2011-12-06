/*
 * simpletest -- simple demo of roombalib
 *
 * http://hackingroomba.com/
 *
 * Copyright (C) 2006, Tod E. Kurt, tod@todbot.com
 *
 * Updates:
 * 14 Dec 2006 - added more functions to roombalib
 */
#include <stdio.h>    /* Standard input/output definitions */
#include <string.h>   /* String function definitions */
#include <pthread.h>
#include "roombalib.h"
#include <semaphore.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#define PI 3.1415926

#define NANOS_PER_MS(n) (n*1000000)

/*'w':Move forward*/
/*'s':Move backward*/
/*'a':Move left */
/*'d':Move right*/
/*'p':Stop*/
/*'e':exit*/
//"q": query
// distance offset 12-13
//angle offset 14-15

typedef struct
{
  char command;
  //  int duration;
} Roomba_Command;

typedef struct
{
  double x; // mm
  double y; // mm 
  double theta; // radians
} position_t; 

position_t pos;

Roomba_Command cmd;

pthread_mutex_t _lock;
pthread_mutex_t* lock = &_lock;

void *exc_cmd(void* _roomba)
{
  
  while(1)
    {
      int err = 0;
      //      err = sem_wait(sem_exc_cmd);
      pthread_mutex_lock(lock);
      if (err != 0) 
	{
	  printf("err: %d\n",err);
	  perror("exc sem wait");
	  exit(1);
	}
      
      switch(cmd.command)
	{
	  
	case'w':
	  roomba_forward(_roomba);
	  break;

	case's':
	  roomba_backward(_roomba);
	  break;

	case'a':
	  roomba_spinleft(_roomba);
	  break;

	case'd':
	  roomba_spinright(_roomba);
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
	  
	  double dist = dist_i;
	  double angle = angle_i / 360.0 * 2.0 * PI;
	  
	  pos.theta += angle;
	  
	  pos.x += dist * cos(pos.theta);
	  pos.y += dist * sin(pos.theta);
	  
	  /*printf("delta dist: %f"  " delta angle: %f"  "\n",
	    dist,angle);*/
	  
	  break;
	}
      
      cmd.command = 'q';
      
      //      sem_post(sem_ready_for_cmd);
      pthread_mutex_unlock(lock);
      
    }
  pthread_exit(NULL);
}

void *get_cmd(void* _roomba)
{
  char buf[16];
  while(1)
    {

      
      // sem_wait(sem_ready_for_cmd);
      // input = getchar();
      // getchar(); 
      // filter the enter button
      
      read(0, buf, 16);
      char input = buf[0];
      
      printf("recv command : %c\n", input);
      
      if (input == 'q')
	{
	  printf("x: %f y: %f theta: %f\n", pos.x, pos.y, pos.theta);
	}
      else if (input == 'r')
	{
	  pthread_mutex_lock(lock);
	  pos.x = 0.0;
	  pos.y = 0.0;
	  pos.theta = 0.0;
	  pthread_mutex_unlock(lock);
	}
      else
	{
	  pthread_mutex_lock(lock);
	  cmd.command = input;
	  pthread_mutex_unlock(lock);
	}

    }
  pthread_exit(NULL);
}

// void* more stuff

int main(int argc, char *argv[]) 
{
  char* serialport;
  int ret;
  
  if( argc>1 && strcmp(argv[1],"-p" )==0 ) {
    serialport = argv[2];
  } else {
    fprintf(stderr,"usage: simpletst -p serialport\n");
    return 0;
  }
  
  roombadebug = 1;
  
  Roomba* roomba = roomba_init( serialport );
  
  ret = 0;
  ret=pthread_mutex_init(lock, NULL);
  if(ret) perror("mutex init");
  
  pthread_t tget_cmd, texc_cmd;
  
  ret=pthread_create(&tget_cmd,NULL,get_cmd,(void *)roomba);
  if(ret)
    {
      printf("ERROR;return code is %d\n",ret);
      exit(-1);
    }    
  
  ret=pthread_create(&texc_cmd,NULL,exc_cmd,(void *)roomba);
  if(ret)
    {
      printf("ERROR;return code is %d\n",ret);
      exit(-1);
    }

  printf("threads go");
  
  pthread_join(texc_cmd, NULL);
  
  roomba_close( roomba );
  roomba_free( roomba );
  
  //  sem_close(sem_exc_cmd);
  //  sem_close(sem_ready_for_cmd);
  
  return 0;
}

