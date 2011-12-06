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

/*'w':Move forward*/
/*'s':Move backward*/
/*'a':Move left */
/*'d':Move right*/
/*'p':Stop*/
typedef struct
{
	char		command;
	int			duration;
}Roomba_Command;

typedef struct position
{
  double x;
  double y;
  double theta;
} position_t; 
  
Roomba_Command cmd;
int cur_cmd = 0;
sem_t sem_exc_cmd;
//sem_t sem_get_cmd;

sem_t ready_cmd;

void *exc_cmd(void* _roomba)
{
  
	while(1)
	{
		sem_wait(&sem_exc_cmd);
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
				break;
		}
	}
	pthread_exit(NULL);
}

void *get_cmd(void* _roomba)
{
	char input;
	while(1)
	{
		input = getchar();
		getchar();// filter the enter button
		printf("recv command :%c\n", input);
		cmd.command = input;
		sem_post(&sem_exc_cmd);
	}
	pthread_exit(NULL);
}

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
    sem_init(&sem_exc_cmd, 0, 0);
		
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

		pthread_join(texc_cmd, NULL);

    roomba_close( roomba );
    roomba_free( roomba );
	
    return 0;
}


