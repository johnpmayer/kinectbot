#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include     <errno.h>
#include     <time.h>  

#define BAUDRATE B57600  
#define DEVICE "/dev/ttyUSB1"  
#define _POSIX_SOURCE 1  
#define FALSE 0  
#define TRUE 1  

typedef struct Roomba_struct {
    int fd;
    char portpath[80];
    unsigned char sensor_bytes[26];
    int velocity;
} Roomba;

int init_SerialPort(char *path)
{
	int fd = open(path, O_RDWR);
	if (-1 == fd) 
	{ 
		  perror("Can't Open Serial Port");  
		  return -1;  
	} 
	else
	{
		struct termios opt;
		if(0 != tcgetattr(fd, &opt))
		{
			perror("Fail to get terminal attribution!");  
			return -1;;
		}
		cfsetispeed(&opt, BAUDRATE);	//set baudrate
		cfsetospeed(&opt, BAUDRATE);	//set baudrate
		opt.c_cflag |= CS8;	//data bits: 8
		opt.c_cflag &= ~PARENB;	//no parity
		opt.c_cflag &= ~CSTOPB;	//stop bits: 1 
		int ret = tcsetattr(fd, TCSANOW, &opt);  
		if (ret != 0)
		{
			perror("Fail to set terminal attribution!");  
			return -1;
		}
		return fd;
	}   
} 
void roomba_drive( Roomba* roomba, int velocity, int radius )
{
    unsigned char vhi = velocity >> 8;
    unsigned char vlo = velocity & 0xff;
    unsigned char rhi = radius   >> 8;
    unsigned char rlo = radius   & 0xff;
    unsigned char cmd[] = { 137, vhi,vlo, rhi,rlo };  // DRIVE
    int n = write(roomba->fd, cmd, 5);
    if( n!=5 )
        perror("roomba_drive: couldn't write to roomba");
}

void roomba_forward( Roomba* roomba )
{
    roomba_drive( roomba, roomba->velocity, 0x8000 );  // 0x8000 = straight
}

int main(int argc, char **argv) 
{  
 	char* serialport;
  if( argc>1 && strcmp(argv[1],"-p" )==0 ) {
      serialport = argv[2];
  } else {
      fprintf(stderr,"usage: simpletst -p serialport\n");
      return 0;
  }
	int fd;
	fd = init_SerialPort(serialport);
	Roomba roomba;
	roomba.fd = fd;
	roomba.velocity = 200;
	if (-1 == fd) 
	{  
		printf("Serial Port Initialization Failed!\n"); 
		exit(-1); 
	} 
	printf("Serial Port Initialized!\n");

	//start roomba
	char buf[] = {128};
	int ret = write(fd, buf, 1); //Send command
	if (ret > 0) 
	{  
		printf("Send %d bytes data\n", ret);
	}  

	//Take control
	buf[0] = 130;
	ret = write(fd, buf, 1); //Send command
	if (ret > 0) 
	{  
		printf("Send %d bytes data\n", ret);
	}  
	
	roomba_forward( &roomba );
	sleep(3);
	//power mode: sleep
	buf[0] = 133;
	ret = write(fd, buf, 1); //Send command
	if (ret > 0) 
	{  
		printf("Send %d bytes data\n", ret);
	}  
	close(fd);
	exit(0);
} 

