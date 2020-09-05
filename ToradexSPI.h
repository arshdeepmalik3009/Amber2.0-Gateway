#pragma once
/*
    spidevlib.c - A user-space program to comunicate using spidev.
                Gustavo Zamboni
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static const char* device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;
//char buf[10];
//char buf2[10];
//int com_serial;
//int failcount;
extern int fd;
//struct spi_ioc_transfer xfer[2];

void print_usage(const char* prog);
void parse_opts(int argc, char* argv[]);
void pabort(const char* s);
uint8_t readRegister(int add1);
void writeRegister(int add1, uint8_t value);
