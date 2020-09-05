//#include <stdio.h>
//#include <unistd.h>
//#include <wiringPi.h>
//#include <wiringPiSPI.h>
//#include <pigpio.h>
#include "gps.h"

#define SPI_CMD_RESET   1
#define SPI_CMD_SEND    2
#define SPI_RES_START   3
#define SPI_RES_STOP    4
#define SPI_RES_INVALID 5

TinyGPSPlus gps;

int gps_fix_get(double lat, double lng, unsigned int sats);
void displayInfo();
static void cleanup();
static void on_user_abort(int arg);

//int gps_fix_get(double lat, double lng, unsigned int sats)
//{
//    int gps_fix = 0;
//    static double lat_old = 0.0;
//    static double lng_old = 0.0;
//    static int rep = 0;
//
//    if ((lat != lat_old) || (lng != lng_old)) {
//        if (sats)
//            gps_fix = 1;
//        else
//            gps_fix = 2;
//        lat_old = lat;
//        lng_old = lng;
//        rep = 0;
//    } else if ((lat_old == (double)0.0) && (lng_old == (double)0.0)) {
//        gps_fix = 0;
//        rep = 0;
//    } else if (++rep >= 2) {
//        gps_fix = 0;
//        rep = 1;
//    } else {
//        if (sats)
//            gps_fix = 1;
//        else
//            gps_fix = 2;
//    }
//
//	return gps_fix;
//}
//
//void displayInfo()
//{
//    int gps_fix = gps_fix_get(gps.location.lat(), gps.location.lng(), gps.satellites.value());
//    printf("gps_fix: %d", gps_fix);
//
//    printf("  Location: ");
//    if (gps.location.isValid())
//    {
//        printf("%.6f,", gps.location.lat());
//        printf("%.6f,", gps.location.lng());
//    }
//    else
//    {
//        printf("INVALID");
//    }
//
//    printf("  Satelites: %d", gps.satellites.value());
//    printf("\n");
//}
//
//static void cleanup()
//{
//    printf("Cleaning up ... ");
//    uart_close();
//    puts("done");
//}
//


void gps_uart_setup()
{
	char port[256];
	strcpy(port, PORT_WRITE);
	if (uart_init(port, B115200)) {
		exit(1);
	}
	const char *gps_init = "AT+QGPS=1\r";
	uart_send((void *)gps_init, strlen(gps_init));
	uart_close();
	sleep(1);
	strcpy(port, PORT_READ);
	if (uart_init(port, B115200)) {
		exit(1);
	}
//    digitalWrite(GPS_CS, 1);
//    pinMode(GPS_CS, OUTPUT);
}

int gps_get(gps_data_t *gps_data)
{

	bool lock = false;
	char c;
	size_t i = 0;
	char n = 0;
	while (uart_data_available()) {
		if (uart_get(&c, 1))
			if (gps.encode(c)){
				if(gps.location.isValid()){
					lock = true;
				    gps_data->lat = gps.location.lat();
				    gps_data->lon = gps.location.lng();
				    gps_data->speed_mph = gps.speed.mph();
				    gps_data->course_deg = gps.course.deg();
				    gps_data->altitude_feet = gps.altitude.feet();
				    gps_data->satellites = gps.satellites.value();
				    gps_data->hdop = gps.hdop.value();
				}
			}
//				if (++n>10) {
//					printf("i: %ld\n",++i);
//					displayInfo();
//					n = 0;
//		}
	}
	if(!lock){
		gps_invalid(gps_data);
	}


//    unsigned char c;
//    unsigned int i;
//
//  //  digitalWrite(GPS_CS, 0);    //enable SPI GPS
//
//    //reset
//    for (i=0; i<3; i++) {
//        c = SPI_CMD_RESET;
//        wiringPiSPIDataRW(SPI_CHANNEL_GPS, &c, 1);
//        if (c == SPI_RES_START)
//            break;
//        usleep(10000);
//    }
//
//    if (c != SPI_RES_START) {
//        digitalWrite(GPS_CS, 1);    //disable SPI GPS
//        return 1;   //start byte issue
//    }
//
//    c = SPI_CMD_SEND;
//    wiringPiSPIDataRW(SPI_CHANNEL_GPS, &c, 1);
//
//    //get data
//    for (i=0; i<sizeof(gps_data_t); i++) {
//        c = SPI_CMD_SEND;
//        usleep(10000);
//        wiringPiSPIDataRW(SPI_CHANNEL_GPS, &c, 1);
//        ((unsigned char *) gps_data)[i] = c;
//    }
//
//    //get postfix
//    c = SPI_CMD_SEND;
//    usleep(10000);
//    wiringPiSPIDataRW(SPI_CHANNEL_GPS, &c, 1);
//    if (c != SPI_RES_STOP) {
//        digitalWrite(GPS_CS, 1);    //disable SPI GPS
//        return 2;   //stop byte issue
//    }
//
//    digitalWrite(GPS_CS, 1);    //disable SPI GPS
    return 0;
}

void gps_invalid(gps_data_t *gps_data)
{
    gps_data->lat = -1.0;
    gps_data->lon = -1.0;
    gps_data->speed_mph = -1.0;
    gps_data->course_deg = -1.0;
    gps_data->altitude_feet = -1.0;
    gps_data->satellites = -1;
    gps_data->hdop = -1;
}
