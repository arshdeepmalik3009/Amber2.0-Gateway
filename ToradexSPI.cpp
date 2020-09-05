/*
 * ToradexSPI.cpp
 *
 *  Created on: Sep 2, 2020
 *      Author: arsh
 */
#include "ToradexSPI.h"

int fd;
struct spi_ioc_transfer xfer[2];

void pabort(const char* s)
{
    perror(s);
    abort();
}

void print_usage(const char* prog)
{
    printf("Usage: %s [-DsbdlHOLC3]\n", prog);
    puts("  -D --device   device to use (default /dev/spidev1.1)\n"
        "  -s --speed    max speed (Hz)\n"
        "  -d --delay    delay (usec)\n"
        "  -b --bpw      bits per word \n"
        "  -l --loop     loopback\n"
        "  -H --cpha     clock phase\n"
        "  -O --cpol     clock polarity\n"
        "  -L --lsb      least significant bit first\n"
        "  -C --cs-high  chip select active high\n"
        "  -3 --3wire    SI/SO signals shared\n");
    exit(1);
}

void parse_opts(int argc, char* argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "device",  1, 0, 'D' },
            { "speed",   1, 0, 's' },
            { "delay",   1, 0, 'd' },
            { "bpw",     1, 0, 'b' },
            { "loop",    0, 0, 'l' },
            { "cpha",    0, 0, 'H' },
            { "cpol",    0, 0, 'O' },
            { "lsb",     0, 0, 'L' },
            { "cs-high", 0, 0, 'C' },
            { "3wire",   0, 0, '3' },
            { "no-cs",   0, 0, 'N' },
            { "ready",   0, 0, 'R' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
        case 'D':
            device = optarg;
            break;
        case 's':
            speed = atoi(optarg);
            break;
        case 'd':
            delay = atoi(optarg);
            break;
        case 'b':
            bits = atoi(optarg);
            break;
        case 'l':
            mode |= SPI_LOOP;
            break;
        case 'H':
            mode |= SPI_CPHA;
            break;
        case 'O':
            mode |= SPI_CPOL;
            break;
        case 'L':
            mode |= SPI_LSB_FIRST;
            break;
        case 'C':
            mode |= SPI_CS_HIGH;
            break;
        case '3':
            mode |= SPI_3WIRE;
            break;
        case 'N':
            mode |= SPI_NO_CS;
            break;
        case 'R':
            mode |= SPI_READY;
            break;
        default:
            print_usage(argv[0]);
            break;
        }
    }
}


//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////

uint8_t readRegister(int add1)
{
    int ret;

    uint8_t tx[] = {
        0x00, 0x00,
    };
    tx[0] = add1;
    uint8_t rx[ARRAY_SIZE(tx)] = { 0, };
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = ARRAY_SIZE(tx),
        //.speed_hz = speed,
        .speed_hz = 5000000,
		.delay_usecs = delay,
        .bits_per_word = bits,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        pabort("can't send spi message");

//    for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
//        if (!(ret % 6)){}
//            puts("");
//        printf("%.2X ", rx[ret]);
//    }
//    puts("");

    return rx[1];
}

//////////
// Write n bytes int the 2 bytes address add1 add2
//////////
void writeRegister(int add1, uint8_t value)
{
    int ret;

    uint8_t tx[] = {
        0x00, 0x00,
    };
    tx[0] = add1 | 0x80;
    tx[1] = value;
    uint8_t rx[ARRAY_SIZE(tx)] = { 0, };
    struct spi_ioc_transfer tr = {
        tx_buf : (unsigned long)tx,
        rx_buf : (unsigned long)rx,
        len : ARRAY_SIZE(tx),
        //.speed_hz = speed,
        speed_hz : 5000000,
		delay_usecs : delay,
        bits_per_word : bits,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        pabort("can't send spi message");

//    for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
//        if (!(ret % 6)){}
//            //puts("");
//        //printf("%.2X ", rx[ret]);
//    }
    //puts("");
}
