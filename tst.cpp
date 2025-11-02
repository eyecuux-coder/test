#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <stdio.h>

#define PIN_DC  25   // GPIO25 → DC
#define PIN_RST 24   // GPIO24 → RST

static void gpio_write(const char* p,const char* v){int f=open(p,O_WRONLY); if(f>=0){write(f,v,strlen(v)); close(f);} }
static void gpio_export(int pin){ char p[64]; snprintf(p,64,"/sys/class/gpio/gpio%d",pin);
  if(access(p,F_OK)!=0){ int f=open("/sys/class/gpio/export",O_WRONLY); char b[8]; sprintf(b,"%d",pin); write(f,b,strlen(b)); close(f); usleep(20000);}
  snprintf(p,64,"/sys/class/gpio/gpio%d/direction",pin); gpio_write(p,"out"); }
static void gpio_set(int pin,int v){ char p[64]; snprintf(p,64,"/sys/class/gpio/gpio%d/value",pin); gpio_write(p, v?"1":"0"); }

static void spi_tx(int fd,const uint8_t* d,size_t n){ write(fd,d,n); }
static void cmd(int fd,uint8_t c){ gpio_set(PIN_DC,0); spi_tx(fd,&c,1); }
static void data1(int fd,uint8_t d){ gpio_set(PIN_DC,1); spi_tx(fd,&d,1); }
static void dataN(int fd,const uint8_t* d,size_t n){ gpio_set(PIN_DC,1); spi_tx(fd,d,n); }

static void set_window(int fd, uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1){
  cmd(fd,0x15); uint8_t c[]={x0,x1}; dataN(fd,c,2);
  cmd(fd,0x75); uint8_t r[]={y0,y1}; dataN(fd,r,2);
  cmd(fd,0x5C);
}

int main(){
  // GPIO
  gpio_export(PIN_DC); gpio_export(PIN_RST);
  gpio_set(PIN_RST,0); usleep(20000);
  gpio_set(PIN_RST,1); usleep(20000);

  // SPI
  int fd=open("/dev/spidev0.0",O_RDWR);
  if(fd<0){perror("spidev"); return 1;}
  uint8_t mode=SPI_MODE_0; uint32_t speed=16000000;
  ioctl(fd,SPI_IOC_WR_MODE,&mode);
  ioctl(fd,SPI_IOC_WR_MAX_SPEED_HZ,&speed);

  // ---------- Waveshare SSD1351 init (128x128) ----------
  cmd(fd,0xFD); data1(fd,0x12);        // Command Lock
  cmd(fd,0xFD); data1(fd,0xB1);        // Command Lock (A2,B1 unlock)
  cmd(fd,0xAE);                        // Display OFF
  cmd(fd,0xB3); data1(fd,0xF1);        // Clock Div
  cmd(fd,0xCA); data1(fd,0x7F);        // MUX = 127
  cmd(fd,0xA0); data1(fd,0x74);        // Set Remap & Color Depth (RGB, 65k)
  cmd(fd,0x15); data1(fd,0x00); data1(fd,0x7F); // Column
  cmd(fd,0x75); data1(fd,0x00); data1(fd,0x7F); // Row
  cmd(fd,0xA1); data1(fd,0x00);        // Start line
  cmd(fd,0xA2); data1(fd,0x00);        // Display offset
  cmd(fd,0xAB); data1(fd,0x01);        // Function select: external VDD
  cmd(fd,0xB1); data1(fd,0x32);        // Precharge
  cmd(fd,0xB4); uint8_t vsl[]={0xA0,0xB5,0x55}; dataN(fd,vsl,3); // Segment low voltages
  cmd(fd,0xC1); uint8_t contrast[]={0xC8,0x80,0xC8}; dataN(fd,contrast,3);
  cmd(fd,0xC7); data1(fd,0x0F);        // Master contrast
  cmd(fd,0xB8); uint8_t gray[]={0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11};
  dataN(fd,gray,16); cmd(fd,0xB9);     // Use default gray table
  cmd(fd,0xA6);                        // Normal display
  cmd(fd,0xAF);                        // Display ON
  usleep(20000);

  // ---------- Full RED fill (RGB565 0xF800) ----------
  set_window(fd,0,0,127,127);
  const uint16_t RED = 0xF800;
  uint8_t two[2]; two[0]=RED>>8; two[1]=RED&0xFF;
  for(int i=0;i<128*128;i++) dataN(fd,two,2);

  close(fd);
  return 0;
}
