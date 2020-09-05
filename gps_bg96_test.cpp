//#include <stdio.h>
//#include <stdlib.h>
//#include <signal.h>
//#include <string.h>
//#include <unistd.h>
//#include "TinyGPS++.h"
//#include "uart.h"
//
//#define PORT_WRITE "/dev/ttyUSB2"
//#define PORT_READ  "/dev/ttyUSB1"
//char port[256];
//
//// The TinyGPS++ object
//TinyGPSPlus gps;
//
//int gps_fix_get(double lat, double lng, unsigned int sats);
//void displayInfo();
//static void cleanup();
//static void on_user_abort(int arg);
//
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
//static void on_user_abort(int arg)
//{
//    cleanup();
//    exit(1);
//}
//
//int main(int argc, char *argv[])
//{
//    printf("%s started\n", argv[0]);
///*
//    if ((argc > 1) && (strlen(argv[1]) + 1 <= sizeof port))
//        strcpy(port, argv[1]);
//    else
//*/
//    strcpy(port, PORT_WRITE);
//    if (uart_init(port, B115200)) {
//        exit(1);
//    }
//
//    const char *gps_init = "AT+QGPS=1\r";
//    uart_send((void *)gps_init, strlen(gps_init));
//
//    uart_close();
//
//    sleep(1);
//
//    strcpy(port, PORT_READ);
//    if (uart_init(port, B115200)) {
//        exit(1);
//    }
//
//	// Catch CTRL-C
//	signal(SIGINT, on_user_abort);
//
//    char c;
//    size_t i = 0;
//    char n = 0;
//    while (1) {
//        if (uart_get(&c, 1))
//            if (gps.encode(c))
//        if (++n>10) {
//            printf("i: %ld\n",++i);
//            displayInfo();
//            n = 0;
//        }
//    }
//
//    cleanup();
//    return 0;
//}
