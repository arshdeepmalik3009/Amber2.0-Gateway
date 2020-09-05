#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include "TinyGPS++.h"
#include "uart.h"


#define SPI_CHANNEL_GPS 0
#define GPS_CS 3
#define PORT_WRITE "/dev/ttyUSB3"
#define PORT_READ  "/dev/ttyUSB1"

//on arduino double is only 4 bytes therefore float is used here

#pragma pack(1)
typedef struct {
	float lat;
	float lon;
	float speed_mph;
	float course_deg;
	float altitude_feet;
	int32_t satellites;
	int32_t hdop;
} gps_data_t;
#pragma pack()

void gps_uart_setup();
int gps_get(gps_data_t *gps_data);
void gps_invalid(gps_data_t *gps_data);
