#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <linux/types.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <signal.h>

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000
#define GPIO_SIZE  0x00000FFF

#define GPIO_OE 		0x134
#define GPIO_SETDATAOUT 	0x194
#define GPIO_CLEARDATAOUT 	0x190

#define ADNS_3080_RESET		(1 << 17)

#define RESET_HIGH()	*gpio_setdataout_addr = (ADNS_3080_RESET)
#define RESET_LOW()	*gpio_cleardataout_addr = (ADNS_3080_RESET)

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_INV_PRODUCT_ID        0x3F

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

int mem_v = 0;
volatile void *gpio_addr = NULL;
volatile uint32_t *gpio_setdataout_addr = NULL;
volatile uint32_t *gpio_cleardataout_addr = NULL;
volatile uint32_t *gpio_oe_addr = NULL;

static const char *device = "/dev/spidev1.0";
static uint8_t mode = SPI_MODE_3;
static uint8_t bits = 8;
static uint32_t speed = 200000;
static uint16_t delay = 75;

static void pabort(const char *s)
{
	perror(s);
	abort();
}


uint8_t spi_read(int fd, uint8_t reg, uint16_t length){
	uint8_t * tx = malloc(length);
	uint8_t * rx = malloc(length);
	*tx = reg;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = length,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	int i;
	for (i = 0; i < length; i++){
		printf("0x%X ",*(rx+i));	
	}
	printf("\r\n");
	
	free(tx);
	free(rx);
	return 0;	
}

uint8_t spi_read_reg(int fd, uint8_t reg){
	uint8_t tx = reg;
	uint8_t rx = 0;
	tx = reg;
	tx &= 0x7F;				// ensure bit 7 is 0
	struct spi_ioc_transfer tr[2];		// send two transfers
	tr[0].tx_buf = (unsigned long)&tx;
	tr[0].rx_buf = NULL;
	tr[0].len = 1;
	tr[0].delay_usecs = 75;			// 75us delay from reg to data
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;
	tr[0].cs_change = 0;			// hold chip select low

	tr[1].tx_buf = NULL;
	tr[1].rx_buf = (unsigned long)&rx;
	tr[1].len = 1;
	tr[1].delay_usecs = 75;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;
	tr[1].cs_change = 1;

	int ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr[0]);
	if (ret < 1)
		pabort("can't send spi message");

	return rx;	
}

void spi_read_data(int fd, uint8_t reg, uint8_t rxbuffer[], uint8_t length){
	uint8_t tx = reg;
	tx &= 0x7F;		// ensure bit 7 is 0

	struct spi_ioc_transfer tr[2];		// send two transfers
	tr[0].tx_buf = (unsigned long)&tx;
	tr[0].rx_buf = NULL;
	tr[0].len = 1;
	tr[0].delay_usecs = 75;			// 75us delay from reg to data
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;
	tr[0].cs_change = 0;			// hold chip select low

	tr[1].tx_buf = NULL;
	tr[1].rx_buf = (unsigned long)&rxbuffer[0];
	tr[1].len = length;
	tr[1].delay_usecs = 75;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;
	tr[1].cs_change = 1;

	int ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr[0]);
	if (ret < 1)
		pabort("can't send spi message");
}

void spi_write_reg(int fd, uint8_t reg, uint8_t data){
	uint8_t tx_reg = reg;
	uint8_t tx_data = data;
	// set msb to indicate write
	tx_reg |= 0x80;

	struct spi_ioc_transfer tr[2];		// send two transfers
	tr[0].tx_buf = (unsigned long)&tx_reg;
	tr[0].rx_buf = NULL;
	tr[0].len = 1;
	tr[0].delay_usecs = 75;			// 75us delay from reg to data
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;
	tr[0].cs_change = 0;			// hold chip select low

	tr[1].tx_buf = (unsigned long)&tx_data;
	tr[1].rx_buf = NULL;
	tr[1].len = 1;
	tr[1].delay_usecs = 75;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;
	tr[1].cs_change = 1;

	int ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr[0]);
	if (ret < 1)
		pabort("can't send spi message");
}

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

int main(void){
	mem_v = open("/dev/mem", O_RDWR);
	gpio_addr = mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_v, GPIO1_BASE);
	if (gpio_addr == MAP_FAILED){
		printf("GPIO mapping failed\r\n");
	      	return 1;
    	}
  	gpio_oe_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_OE);
  	gpio_setdataout_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_SETDATAOUT);
  	gpio_cleardataout_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_CLEARDATAOUT);

	// configure P9.23 as output
	*gpio_oe_addr = ~ADNS_3080_RESET;
	// reset chip
	RESET_HIGH();
	usleep(100000);
	RESET_LOW();
	usleep(200000);
	int spi_fd = open(device, O_RDWR);
	if (spi_fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	int ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	
	usleep(200000);

	uint8_t pid = spi_read_reg(spi_fd, ADNS3080_PRODUCT_ID);
	printf("Device PID: 0x%02X\r\n", pid);
	usleep(50);

	uint8_t config = spi_read_reg(spi_fd, ADNS3080_INV_PRODUCT_ID);
	printf("Inverse PID: 0x%02X\r\n", config);
	usleep(50);

	config = spi_read_reg(spi_fd, ADNS3080_CONFIGURATION_BITS);
	printf("Default configuration: 0x%02X\r\n", config);
	usleep(50);

	spi_write_reg(spi_fd, ADNS3080_CONFIGURATION_BITS, config | (1 << 4));
	usleep(50);
	config = spi_read_reg(spi_fd, ADNS3080_CONFIGURATION_BITS);
	printf("Updated configuration: 0x%02X\r\n", config);
	usleep(50);

	uint8_t buffer[10] = {0};
	while(keepRunning){
		spi_read_data(spi_fd, ADNS3080_MOTION_BURST, buffer, 4);
		printf("m: 0x%02X x: %4d y:%4d q:%u\r\n", buffer[0], (int8_t)buffer[1], (int8_t)buffer[2], buffer[3]);
		usleep(100000);
	}
	close(mem_v);
	close(spi_fd);
	return 0;
}
