#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DC 25
#define RST 24

void gpio_set(int pin, int val) {
    char b[64];
    sprintf(b, "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(b, O_WRONLY); if (fd >= 0) { write(fd, val ? "1" : "0", 1); close(fd); }
}
void gpio_exp(int pin) {
    char b[64];
    sprintf(b, "/sys/class/gpio/gpio%d", pin);
    if (access(b, F_OK) != 0) {
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        sprintf(b, "%d", pin); write(fd, b, strlen(b)); close(fd);
        usleep(20000);
    }
    sprintf(b, "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(b, O_WRONLY); write(fd, "out", 3); close(fd);
}

void cmd(int fd, uint8_t c) {
    gpio_set(DC, 0); write(fd, &c, 1);
}
void data8(int fd, uint8_t d) {
    gpio_set(DC, 1); write(fd, &d, 1);
}

int main() {
    gpio_exp(DC); gpio_exp(RST);
    gpio_set(RST, 0); usleep(20000);
    gpio_set(RST, 1); usleep(20000);

    int fd = open("/dev/spidev0.0", O_RDWR);
    uint8_t mode = SPI_MODE_0; ioctl(fd, SPI_IOC_WR_MODE, &mode);
    uint32_t speed = 8000000; ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    cmd(fd, 0xAF); // display ON
    cmd(fd, 0x15); data8(fd, 0); data8(fd, 95);
    cmd(fd, 0x75); data8(fd, 0); data8(fd, 63);
    cmd(fd, 0x5C);

    for (int i = 0; i < 96 * 64; i++) {
        uint16_t red = 0xF800;
        data8(fd, red >> 8); data8(fd, red & 0xFF);
    }
    return 0;
}
