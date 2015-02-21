#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>
#include <time.h>

#define OUTPUT          1
#define INPUT           0
#define INPUT_PULLUP    2
#define HIGH            1
#define LOW             0

int uart0_filestream = -1;
unsigned char tx_buffer[256];
unsigned char dat[256];
unsigned char dat1[256];
unsigned char *p_tx_buffer;
unsigned short int co = 0,ptr = 0;

void myWrite(unsigned char tmp2[])
{
  int i = 0;
  p_tx_buffer = &tx_buffer[0];
  while(tmp2[i] != '\0')
  {
    *p_tx_buffer++ = tmp2[i];
    i++;
  }
  *p_tx_buffer++ = '\0';
  if (uart0_filestream != -1)
  {
    int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));              //Filestream, bytes to write, number of bytes to write
    //printf("\nData sent %s", tmp2);
    if (count < 0)
    {
      printf("UART TX error\n");
    }
  }
}


void myRead()
{
  int flag = 0;
  co = 0;
  if (uart0_filestream != -1)
  {
    while(flag == 0)
    {
      unsigned char rx_buffer[256];
      int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);          //Filestream, buffer to store in, number of bytes to read (max)
      if (rx_length < 0)
      {
      }
      else if (rx_length == 0)
      {
      }
      else
      {
        int k = 0;
        for(k; k < rx_length; k++)
        {
          dat[co] = rx_buffer[k];
          co++;
          if(rx_buffer[k] == '\0')
          {
            flag = 1;
            //printf("\nData recieved = %s",dat);
            break;
          }
        }
      }
    }
  } 
}

void delay(int val)
{
    dat1[ptr] = '\0';
    myWrite(dat1);
    myRead();
    usleep(val*1000);
    dat[0] = 'c';
    ptr = 1;
}

void microDelay(int val)
{
    dat1[ptr] = '\0';
    myWrite(dat1);
    myRead();
    usleep(val);
    dat1[0] = 'c';
    ptr = 1;
}


void pinMode(char pin,char val)
{
  dat1[ptr++] = ++pin;
  dat1[ptr++] = ++val;
}

extern void setup();

float getMag()
{
  char tmp_mag[20];
  int m = 0;
  float val = 0;
  dat1[0] = 'b';
  dat1[1] = '0';
  dat1[2] = 0;
  myWrite(dat1);
  myRead();
  if(dat[1] == '1')
  {
    tmp_mag[m] = dat[m+2];
    while(tmp_mag[m] != '\0')
    {
      m++;
      tmp_mag[m] = dat[m+2];
    }
    val = atof(tmp_mag);
    return val;
  }
  else
  {
    printf("\nMag read unsuccessful");
  }
}

/*void getEnc1()
{
}

void getEnc2()
{ 
}

void getEnc2()
{ 
}*/


void sensor_update()
{
  dat1[0] = 'b';
  dat1[0] = '0';
  dat1[1] = 0;
  myWrite(dat1);
  myRead();
  //parse();
}

void analogWrite(char pin, char val)
{
  dat1[ptr++] = ++pin;
  dat1[ptr++] = ++val;
}

void digitalWrite(char pin, char val)
{
  dat1[ptr++] = ++pin;
  dat1[ptr++] = ++val;
}

extern void loop();



int main()
{
  printf("Programme Started");
  uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
  if (uart0_filestream == -1)
  {
    //ERROR - CAN'T OPEN SERIAL PORT
    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
  }

  struct termios options;
  tcgetattr(uart0_filestream, &options);
  options.c_cflag = B19200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(uart0_filestream, TCIFLUSH);
  tcsetattr(uart0_filestream, TCSANOW, &options);
  printf("\nRPI UART Initialised");
  
/*  
  unsigned char give[] = "Happy Birthday to you";
  while(1)
  {
    myWrite(give);
    myRead();
    getchar();
  }
*/unsigned char give[] = "e";
  while((dat[0] != 'n')&&(dat[0] != 'i'))
  {
    myWrite(give);
    myRead();
  }
  printf("\nMCU Connected");
  dat1[0] = 'a';
  ptr = 1;
  setup();
  dat1[ptr++] = '\0';
  myWrite(dat1);
  printf("\nSetup done");

  while(1)
  {
    myRead();
    if(dat[0] == 'i')
    {
      //sensor_update();
      dat1[0] = 'c';
      ptr = 1;
      loop();
      dat1[ptr] = '\0';
      myWrite(dat1);
    }
    else
    if(dat[0] == 'n')
    {
      dat1[0] = 'a';
      ptr = 1;
      setup();
      dat1[ptr++] = '\0';
      myWrite(dat1);
      printf("\nSetup done");
    }
  }

  close(uart0_filestream);
  return 0;

}
