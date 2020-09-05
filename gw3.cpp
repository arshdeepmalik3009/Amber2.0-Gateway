//#define TELIT
//#define MQTT
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <curl/curl.h>
#include <ctime>
#include <iomanip>
//#include <wiringPi.h>
//#include <wiringPiSPI.h>
//#include "gps.h"
//#include "conf.h"
#ifdef TELIT
#include "telit.h"
#endif //TELIT
#ifdef MQTT
#include "mqtt_utils.h"
#include "ingest.h"
#endif //MQTT
#include "RF95.h"
#include "json.h"
#include "uart.h"
#include "TinyGPS++.h"
#include <signal.h>
#include "gps.h"
//#include "ble_dfu.h"
#define TELIT_SERIAL "/dev/ttyS0"
#define TRY(f) { int e; if ((e=f)) return e;}

#define TEMPERATURE_LOGGING
#define TEMPERATURE_DECODING
#define SIMPLE_TIME_UPDATE
//#define CONFIG_HUB_WHITELIST
#define CORPORATE_RETARGETING
#define MULTI_AXIS
//#define CAN_ENABLE
#define INT16_BASED_READINGS
//#define HUB_INFO_FROM_CLOUD
#define FORCE_TIME_UPDATE
#define GW_INFO_FROM_CLOUD
//#define QR_ID_UPLOADS

#define DEBUG
#define LOG_FILE_LOCATION    "/home/gateway/Documents/"
#define READ_FILE_LOCATION    "/home/gateway/Documents/conmet_Config.txt"
using namespace std;

#define MAX_HUB_CNT 30
#define HUB_CFG_MAX_LENGTH 240
#define HUB_CFGI_MAX_LENGTH 200
#define CELL_SEND_BUF_MAX_LENGTH 2048
#define CELL_BUF_MAX_LENGTH 512
#define CELL_VAR_MAX_LENGTH 30
#define GPS_TIMEOUT 5
#define LORA_DATA_TIMEOUT 30
#define LORA_HEARTBEAT_TIMEOUT 5*60
#define ENDPOINT_LENGTH 27

char host_and_port[ENDPOINT_LENGTH];
#define HUB_DEFAULT_TIMEOUT 600
#define HUB_DEFAULT_MULTIPLIER 23
#define HUB_DEFAULT_ACC_READ_FLAG 0
#define HUB_DEFAULT_LINE_BUSY_BACKOFF 10
#define HUB_DEFAULT_RETRY_BACKOFF 50
#define HUB_DEFAULT_TEMPERATURE_NOM_TIMEOUT 60
#define HUB_DEFAULT_TEMPERATURE_ALERT_TIMEOUT 60
#define HUB_DEFAULT_SENSOR_SELECTOR 1
#define HUB_DEFAULT_NUMBER_OF_SAMPLES 1024
#define HUB_DEFAULT_MOVEMENT_THRESHOLD 3
#define HUB_DEFAULT_MOVEMENT_READ_THRESHOLD 10
#define HUB_DEFAULT_TEMPERATURE_NOM_MULTIPLIER 60
#define HUB_DEFAULT_TEMPERATURE_ALERT_MULTIPLIER 10
#define HUB_DEFAULT_ACCEL_TIMEOUT 3600
#define HUB_DEFAULT_LORA_POWER 23
#define HUB_DEFAULT_MOVEMENT_HYSTERESIS_UP 1
#define HUB_DEFAULT_MOVEMENT_HYSTERESIS_DOWN 4
#define HUB_DEFAULT_TEMPERATURE_ALERT_THRESH 65
#define HUB_DEFAULT_SAMPLE_FREQUENCY 400


int gps_fix = 0;
char buffer[500];

//#define CHARI
#define HEXI

#define RF95_FREQ 915E6

#define TEST

typedef struct HubRec {
    char hubId[15];
    char hubNo[15];
    char hubQR[20];
    char FwVersion[20];
    int SensorID;
    char MeasId[20];
    int SampleCount;
    int SampleTime;
    int SlotNumber;
    int LoraFreq;
    int LoraPower;
    int SampleFreq;
    int HubFFT;
    int freqThresh;
    int accThresh;
    int dfuBle;
    int temperatureSampleTime;
    int CaptureMode;

} HubRec_t;

typedef struct GwRec {
    int HubCnt;
    char Vid[20];
    char macId[13];
    gps_data_t gps_data;
    float sampleLat = 0.0;
    float sampleLon = 0.0;
    int gps_timeout = 59;
    int SamplePeriod = 0;
    int SlotCount = 0;
    int dfuGo = 0;
    int enCAN = 0;
    HubRec_t HubList[MAX_HUB_CNT];
} GwRec_t;

typedef enum {
    NBUF_CALL_GET,
    NBUF_CALL_POST,
    NBUF_CALL_MQTT
} nbuf_call;

typedef struct {
    nbuf_call call;
    const char *adr;
    void *buf;
    void (*cb)(char *res);
} nbuf_t;

#define NBUF_SIZE 1024
nbuf_t nbuf[NBUF_SIZE];
int nbuf_con = 0;
int nbuf_pro = 0;
GwRec_t Gateway;
RF95 rf95;

//char idBuf[MAX_ID_BUF_LENGTH];
uint8_t buff[RH_RF95_MAX_MESSAGE_LEN];
char bufff[RH_RF95_MAX_MESSAGE_LEN];
char buf[CELL_SEND_BUF_MAX_LENGTH];
char cellBuff[CELL_BUF_MAX_LENGTH];
char varbuf[CELL_VAR_MAX_LENGTH];
char hubCfgbuf[HUB_CFG_MAX_LENGTH];

#ifdef TELIT
static telit_t telit0;
#else
char curl_arg_str[2048];
char curl_ret_str[2048];
#endif //TELIT
char GatewayCfgbuf[200];

#ifdef MQTT
ingest_gps_data_t gps_data;
ingest_temp_data_t temp_data;
ingest_vib_data_t vib_data;
ingest_sensor_status_data_t sensor_status_data;
#endif //MQTT

#ifdef MULTI_AXIS
float loraBufValues[2048*3];
#else //MULTI_AXIS
float loraBufValues[4096];
#endif //MULTI_AXIS
int loraBufTemperatureValues[121];
uint8_t lora_buf[70][255];
uint8_t lora_buf_count[70];
//uint16_t SUB_ID, SUB_LOG_NUM, SUB_BAT, TIMESTAMP, SUB_SAMPLES, SUB_TIME, SUB_MES_NUM;
//float BAT, MUR_TEMP, STS_TEMP, TMP_TEMP;

//int comm_freq;
//float lat = 0.0;
//float lon = 0.0;

char dataMessage[2] = "D";
char dataStartMessage[2] = "S";
char dataEndMessage[2] = "E";
char cfgReqMessage[2] = "R";
char temperatureMessage[2] = "T";
char defMessage[2] = "";

int rtcCountTimeout = 0;
//int timeroutPeriod = SLEEP_TIME_SEC;
int16_t adcResult[5];
float prescise_result = 0;
int timerFlag = 0;
int globalSendCounter = 0;
int logCounterf = 0;
int logCountert = 0;
int timeoutFlagb = 0;
int sleepflag = 0;
bool sendFlag = false;
uint8_t currentCycle = 0;
uint8_t testabc = 0;
int protect = 0;
//AK int noSendCounter = 1;
int noSendCounter = 0;
int last_noSendCounter = 0;
int tempperiod;
int last_tempperiod;
int counter = 0;
int lastRssiLora;
int lastRssiLoraSum;
int lastRssiLoraCount;
int lastSNRLora;
int loraValTemperatureCounter = 0;

//Decoding
int bitsPerMeasurement = 0;
float scaleFactor = 0;

int hubTimeout = HUB_DEFAULT_TIMEOUT;
int hubTimeoutMulti = HUB_DEFAULT_MULTIPLIER;
int hubAccUpload = HUB_DEFAULT_ACC_READ_FLAG;
int hubLineBusyBackoff = HUB_DEFAULT_LINE_BUSY_BACKOFF;
int hubRetryBackoff = HUB_DEFAULT_RETRY_BACKOFF;
bool heartbeatHit = false;
int hubTemperatureTimeout = HUB_DEFAULT_TEMPERATURE_NOM_TIMEOUT;
int hubTemperatureAlertTimeout = HUB_DEFAULT_TEMPERATURE_ALERT_TIMEOUT;
int hubSensorSelector = HUB_DEFAULT_SENSOR_SELECTOR;
int hubNumberOfSamples = HUB_DEFAULT_NUMBER_OF_SAMPLES;
int hubMovementThreshold = HUB_DEFAULT_MOVEMENT_THRESHOLD;
int hubMovementReadThreshold = HUB_DEFAULT_MOVEMENT_READ_THRESHOLD;
int hubTemperatureMultiplier = HUB_DEFAULT_TEMPERATURE_NOM_MULTIPLIER;
int hubTemperatureAlertMultiplier = HUB_DEFAULT_TEMPERATURE_ALERT_MULTIPLIER;
int hubAccelTimeout = HUB_DEFAULT_ACCEL_TIMEOUT;
int hubLoraPower= HUB_DEFAULT_LORA_POWER;
int hubMovementHysteresisUp = HUB_DEFAULT_MOVEMENT_HYSTERESIS_UP;
int hubMovementHysteresisDown = HUB_DEFAULT_MOVEMENT_HYSTERESIS_DOWN;
int hubTemperatureAlertThresh = HUB_DEFAULT_TEMPERATURE_ALERT_THRESH;
int HubSampleFrequency = HUB_DEFAULT_SAMPLE_FREQUENCY;

uint32_t heartbeatTimeout = LORA_HEARTBEAT_TIMEOUT;
time_t time_c = 0;
time_t time_o = 0;

time_t rawtime;
time_t lastrawtime;
time_t lastloratime;
time_t prelogtime;
struct tm * timeinfo;

bool BleOtaFlag = false;
char BleOtaHid[20];

int err;
size_t size;

//prototypes
int receive_GatewayCfg_from_cloud(char *vid, char *gwCfgBuf);
void get_gateway_info();
void parse_gwInfo(char *buf);
bool receive_hubCfg_from_cloud(char *hid, char *gwCfgBuf, int hbSeq);
bool receive_hubQR_from_cloud(char *hid);
void set_hubQR_in_config(char *hid, char *returnStr);
void createHubCfg(char *hid, char *hubCfgbuf);
bool checkForHOTA(char *hid);
void parse_hubInfo(char *buf, int hubSeq, bool initialRun);
void print_gateway_info();
bool send_acceldata_to_cloud(char *hid, int pktCount);
void lora_receiver(void);
void loraBufDecode();
bool SendMeasDataFloats(int sampleCnt, char *measID, char *hid);
bool SendMeasDataFloatsMqtt(int sampleCount, char *measID, char *hid);
int checkHubId(char *hid);
time_t raw_time_get(time_t *sec);
void loraInit(void);
void toJson(char *dest, char *src);
void write_Log(char *logName, char *strNote, const char *strToLog, int utc);
void pingGps();
int nbuf_delta();
int nbuf_add(nbuf_call call, const char *adr, const char *buf, void (*cb)(char *res));
void nbuf_process();
int gps_fix_get(double lat, double lng, unsigned int sats);
int TestBit(uint8_t A[], int k);
int decode_bits_to_array(uint8_t B[], int bit_length, int startBit);
void reboot(void);
#ifdef TELIT
int httpqry(telit_t *telit, const char *param, char *resp, size_t *resp_size);
int httpsnd(telit_t *telit, const char *param, const char *data, size_t data_size,
        char *resp, size_t *resp_size);
int ota(telit_t *telit, const char *ftp_server, const char *ftp_user,
    const char *ftp_pass, const char *local_dir, const char *remote_dir,
    int interval_days, int hour, const char *id);
#endif //TELIT

void reboot(void)
{
    system("reboot");
    while (1) {}
}

int TestBit(uint8_t A[], int k) {
  return ((A[k/8] & (1 << (k%8))) != 0);
}

int decode_bits_to_array(uint8_t B[], int bit_length, int startBit) {
  int decoded_value = 0;
  for(int i = 0; i < bit_length; i++) {
    decoded_value |= (int32_t)(TestBit(B, (startBit+i)) << i);
    if(i == (bit_length-1))
      if(TestBit(B, (startBit+i)))
        decoded_value |= 0xFFFFFFFF << bit_length;
  }
  return decoded_value;
}


void config_read(const char* str, char* buff){
	char buffer[500];
	FILE * stream;
		  stream = fopen("/home/torizon/json_test.txt", "r+");
		  int count = fread(&buffer, sizeof(char), 500, stream);
		  fclose(stream);
		  //printf("count:%d\n", count);
		  //printf("buffer: %s\n", buffer);
		  JSONObject *json = parseJSON(buffer);
	for(int i=0; i<json->count; i++){
		  //printf("Key: %s, Value: %s\n", json->pairs[i].key, json->pairs[i].value->stringValue);
          if(strcmp(json->pairs[i].key, str ) == 0){
        	  strcpy(buff, json->pairs[i].value->stringValue);
          }
	}

}

int gps_fix_get(double lat, double lng, unsigned int sats)
{
    int gps_fix = 0;
    static double lat_old = 0.0;
    static double lng_old = 0.0;
    static int rep = 0;

    if ((lat != lat_old) || (lng != lng_old)) {
        if (sats)
            gps_fix = 1;
        else
            gps_fix = 2;
        lat_old = lat;
        lng_old = lng;
        rep = 0;
    } else if ((lat_old == (double)0.0) && (lng_old == (double)0.0)) {
        gps_fix = 0;
        rep = 0;
    } else if (++rep >= 2) {
        gps_fix = 0;
        rep = 1;
    } else {
        if (sats)
            gps_fix = 1;
        else
            gps_fix = 2;
    }

	return gps_fix;
}

int curl(const char *urlStr)
{
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, urlStr);
        res = curl_easy_perform(curl);

        /* Check for errors */
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                  curl_easy_strerror(res));
            return 1;
        }
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
    return 0;
}

size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

int curlReq(const char *urlStr, char *retStr)
{
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);

    curl = curl_easy_init();
    std::string readBuffer;
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, urlStr);
        //curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        /* Perform the request, res will get the return code */
        res = curl_easy_perform(curl);

        /* Check for errors */
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                  curl_easy_strerror(res));
            return 1;
        }

        strcpy(retStr, readBuffer.c_str());

        /* always cleanup */
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
    return 0;
}

int curlPostReq(const char *urlStr, char *inputStr, char *retStr)
{
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);

    curl = curl_easy_init();
    std::string readBuffer;

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "charsets: utf-8");
    headers = curl_slist_append(headers, "Expect:");

    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, urlStr);
        //curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_POST, 1);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, inputStr);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        /* Perform the request, res will get the return code */
        res = curl_easy_perform(curl);

        /* Check for errors */
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                  curl_easy_strerror(res));
            return 1;
        }

        strcpy(retStr, readBuffer.c_str());

        /* always cleanup */
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
    return 0;
}

void toJson(char *dest, char *src)
{
    char *p;

    strcpy(dest, "{");

    if (src[0] == '?')
        src++;
    p = strtok(src, "=");
    if (p == NULL) {
        strcat(dest, "}");
        return;
    }

    strcat(dest, "\"");
    strcat(dest, p);
    strcat(dest, "\":");

    p = strtok(NULL, "&");
    if (p == NULL) {
        strcat(dest, "}");
        return;
    }

    strcat(dest, "\"");
    strcat(dest, p);
    strcat(dest, "\"");

    for (;;) {
        p = strtok(NULL, "=");
        if (p == NULL) {
            strcat(dest, "}");
            break;
        }

        strcat(dest, ",\"");
        strcat(dest, p);
        strcat(dest, "\":");

        p = strtok(NULL, "&");
        if (p == NULL) {
            strcat(dest, "}");
            break;
        }

        strcat(dest, "\"");
        strcat(dest, p);
        strcat(dest, "\"");
    }
}

void loraInit(void) {

if (!rf95.begin(RF95_FREQ)) {
#ifdef TEST
    printf("LoRa radio init failed");
#endif //TEST
    while (1);
}
#ifdef TEST
  printf("LoRa radio init OK!\n");
  printf("Set Freq to: ");
  printf("%f\n", RF95_FREQ);
#endif //TEST
  rf95.setTxPower(23, 1);
}

int receive_GatewayCfg_from_cloud(char *vid, char *gwCfgBuf) {

#ifdef TEST
    printf("Gettting GateWay config data from cloud\n");
#endif
    printf("%s/getGwayConfig?gway=%s\n", host_and_port, vid);
    char args[2048];
    strcpy(args, "/getGwayConfig?gway=");
    strcat(args, vid);

#ifdef DEBUG
    write_Log("Conmet_GW", "Requesting gateway config", args, 0);
#endif

#ifdef TELIT
    telit0.httpqry(args, telit0.buffer, &size);
#else
    strcpy(curl_arg_str, host_and_port);
    strcat(curl_arg_str, args);
    curlReq(curl_arg_str, curl_ret_str);
    //printf("received config from server = %s\n", curl_ret_str);
#endif //TELIT

    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    for (int i = 0; i < size; i++) {
#ifdef TELIT
        gwCfgBuf[i] = telit0.buffer[i];
#else
        gwCfgBuf[i] = curl_ret_str[i];
#endif //TELIT
        gwCfgBuf[i + 1] = '\n';
    }

#ifndef TELIT
    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    strcpy(gwCfgBuf, curl_ret_str);
#endif //TELIT

#ifdef DEBUG
    write_Log("Conmet_GW", "Receiving gateway config", gwCfgBuf, 0);
#endif

    printf("Config received = %s\n", gwCfgBuf);

#ifdef TEST
    printf("Gettting config data from cloud ... DONE\n");
#endif

    return 0;
}

void get_gateway_info() {
    printf("Getting Gateway info\n");

    // receive_GatewayCfg_from_cloud("FE03FCCA489B",GatewayCfgbuf);
    receive_GatewayCfg_from_cloud(Gateway.macId, GatewayCfgbuf);

    parse_gwInfo(GatewayCfgbuf);

    for (int i = 0; i < Gateway.HubCnt; i++) {
        receive_hubCfg_from_cloud(Gateway.macId, GatewayCfgbuf, i + 1);
        parse_hubInfo(GatewayCfgbuf, i, true);
#ifdef QR_ID_UPLOADS
        receive_hubQR_from_cloud(Gateway.HubList[i].hubId);
#endif //QR_ID_UPLOADS
    }
    print_gateway_info();
    //raw_time_get( &time_o );
}

void parse_gwInfo(char *buf) {
//  char temp_hnum;
//  char temp_hid;
//  int hcnt = 0;
//  int i;
    printf("Inside parse_gwInfo buf length = %u\n", strlen(buf));
    printf("str target = %s \n", buf);
    if (strlen(buf)) {
        char *pch;
        pch = strsep(&buf, ",");

        // ---------------------------
        // Get Vehicle ID
        //----------------------------
        if (pch != NULL) {
            strcpy(Gateway.Vid, pch + 1);
        } else {
            return;
        }

        pch = strsep(&buf, ",");
        // ---------------------------
        // Get Hub Count
        //----------------------------

        if (pch != NULL)
            Gateway.HubCnt = atoi(pch);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.SamplePeriod = atoi(pch); //slotfrequency

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.SlotCount = atoi(pch); //slot count

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.enCAN = atoi(pch);

        printf("\n");
        printf("The number of HUbs = %d\n", Gateway.HubCnt);
        printf("The period in seconds per read = %d\n", Gateway.SamplePeriod);
        printf("The number of slots in period = %d\n", Gateway.SlotCount);

#ifdef GW_INFO_FROM_CLOUD
//        config["GW_VID"] = Gateway.Vid;
//        config_write();
//        config_read();
#endif //GW_INFO_FROM_CLOUD

    }

#ifdef GW_INFO_FROM_CLOUD
    config_read("GW_VID", Gateway.Vid);
   // strcpy(Gateway.Vid, config["GW_VID"].asCString());
#endif //GW_INFO_FROM_CLOUD

}

bool receive_hubQR_from_cloud(char *hid) {

#ifdef TEST
    printf("Gettting hub QR data from cloud\n");
#endif
    char gwCfgBuf[HUB_CFGI_MAX_LENGTH];
    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    sprintf(gwCfgBuf, "/mac-to-qr?hubmacid=%s", hid);

    printf("%s%s\n", host_and_port, gwCfgBuf);

#ifdef DEBUG
    write_Log("Conmet_GW", "Requesting hub QR", gwCfgBuf, 0);
#endif

#ifdef TELIT
    telit0.httpqry(gwCfgBuf, telit0.buffer, &size);
#else
    strcpy(curl_arg_str, host_and_port);
    strcat(curl_arg_str, gwCfgBuf);
    curlReq(curl_arg_str, curl_ret_str);
#endif //TELIT

    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    for (int i = 0; i < size; i++) {
#ifdef TELIT
        gwCfgBuf[i] = telit0.buffer[i];
#else
        gwCfgBuf[i] = curl_ret_str[i];
#endif //TELIT
        gwCfgBuf[i + 1] = '\n';
    }

#ifndef TELIT
    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    strcpy(gwCfgBuf, curl_ret_str);
#endif //TELIT

    printf("QR received = %s\n", gwCfgBuf);

#ifdef DEBUG
    write_Log("Conmet_GW", "Receiving QR config", gwCfgBuf, 0);
#endif

    //set_hubQR_in_config(hid, gwCfgBuf);

#ifdef TEST
    printf("Gettting hub QR data from cloud ... DONE\n");
#endif

    return true;
}

//void set_hubQR_in_config(char *hid, char *returnStr) {
//
//#ifdef TEST
//  printf("QR return str = %s\n", returnStr);
//#endif
//
//  char *poch;
//  if(strlen(returnStr) != 0) {
//      poch = strtok(returnStr,  "\"");
//      if(poch != NULL) {
//        poch = strtok(NULL, "\"");
//        if(poch != NULL) {
//          if(strcmp(poch, "qrcode") == 0) {
//            poch = strtok(NULL,  "\"");
//            if(poch != NULL) {
//              poch = strtok(NULL,  "\"");
//              if(poch != NULL) {
//                config[hid] = poch;
//                config_write();
//              }
//            }
//          }
//        }
//      }
//    }
//    config_read();
//
//#ifdef TEST
//    printf("hub %s QR = %s\n", hid, config[hid].asCString());
//#endif
//
//}

bool receive_hubCfg_from_cloud(char *hid, char *gwCfgBuf, int hbSeq) {
#ifdef TEST
    printf("Gettting hub config data from cloud\n");
#endif
    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    sprintf(gwCfgBuf, "/getHubConfig?gway=%s&hsn=%d", hid, hbSeq);
    printf("%s%s\n", host_and_port, gwCfgBuf);
//    sleep(1);

#ifdef DEBUG
    write_Log("Conmet_GW", "Requesting hub config", gwCfgBuf, 0);
#endif

#ifdef TELIT
    telit0.httpqry(gwCfgBuf, telit0.buffer, &size);
#else
    strcpy(curl_arg_str, host_and_port);
    strcat(curl_arg_str, gwCfgBuf);
    curlReq(curl_arg_str, curl_ret_str);
#endif //TELIT

//repeat
/*
    start_timer(60000);
    while (!timerFlag) {
        checkhttp = gprs.httpIsConnected();
        if (checkhttp == 1)
            break;
    }
    if (timerFlag) {
        printf("http connect: Failed to get config from the cloud\n");
        systemSleep();
        //tempperiod = last_tempperiod;
        //noSendCounter = last_noSendCounter;
        return false;
    }
    stop_timer();
#ifdef TEST
    printf("HTTP connection success to PORT <8096> for comm_freq...\n");
    sleep(1);
#endif //TEST
    gprs.httpRead();

    int ret;
    //while ((ret = gprs.httpIsRead()) == 0);
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
        if ((ret = gprs.httpIsRead()) != 0)
            break;
        if (i == (MAX_ATTEMPTS - 1)) {
            printf("httpIsRead: Failed to get config from the cloud\n");
            systemSleep();
            //tempperiod = last_tempperiod;
            //noSendCounter = last_noSendCounter;
            return false;
        }
    }
*/

    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    for (int i = 0; i < size; i++) {
#ifdef TELIT
        gwCfgBuf[i] = telit0.buffer[i];
#else
        gwCfgBuf[i] = curl_ret_str[i];
#endif //TELIT
        gwCfgBuf[i + 1] = '\n';
    }

#ifndef TELIT
    memset(gwCfgBuf, 0, HUB_CFGI_MAX_LENGTH);
    strcpy(gwCfgBuf, curl_ret_str);
#endif //TELIT

    printf("Config received = %s\n", gwCfgBuf);

#ifdef DEBUG
    write_Log("Conmet_GW", "Receiving hub config", gwCfgBuf, 0);
#endif

#ifdef TEST
    printf("Gettting hub config data from cloud ... DONE\n");
#endif

    return true;
}

void createHubCfg(char *hid, char *hubCfgbuf) {

  int index = Gateway.HubCnt;
  int i;
  int CurrentPeriodCount = 0;

  //rtc.readClock(&tm1);
  //time_c = rtc.mktime(tm1);
  raw_time_get( &time_c );
  raw_time_get( &rawtime );

  CurrentPeriodCount = (time_c - time_o) % Gateway.SamplePeriod;

#ifndef CONFIG_HUB_WHITELIST
  index = 0;
#endif //CONFIG_HUB_WHITELIST


  for (i = 0; i < Gateway.HubCnt; i++) {
    //printf("String 1 = %s\nString 2 = %s\n", Gateway.HubList[i].hubId, hid);
    if (strcmp(Gateway.HubList[i].hubId, hid) == 0) {
      //printf("hub index location = %d\n", i);
      index = i;
      break;
    }
  }

#ifdef SIMPLE_TIME_UPDATE
    sprintf(hubCfgbuf, "%s,U,3,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", hid,
      hubTimeout, hubTimeoutMulti, hubAccUpload, hubLineBusyBackoff, hubRetryBackoff,
      hubTemperatureTimeout, hubSensorSelector, hubNumberOfSamples, hubMovementThreshold,
      hubTemperatureMultiplier, hubAccelTimeout, hubLoraPower, hubMovementHysteresisUp, hubMovementHysteresisDown);
#endif //SIMPLE_TIME_UPDATE


  if (index < Gateway.HubCnt) {

    //    sprintf(hubCfgbuf, "%s,U,3,%d,%d,%ld,%d\n", hid, Gateway.HubList[index].MeasId, Gateway.HubList[index].SensorID,
    //        Gateway.HubList[index].SampleCount, Gateway.HubList[index].SampleTime);
#ifdef SIMPLE_TIME_UPDATE

    sprintf(hubCfgbuf, "%s,U,3,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d\n", hid, hubTimeout, hubTimeoutMulti,
      hubAccUpload, hubLineBusyBackoff, hubRetryBackoff, hubTemperatureTimeout, hubTemperatureAlertTimeout, hubSensorSelector, hubNumberOfSamples,
      hubMovementThreshold, hubMovementReadThreshold, hubTemperatureMultiplier, hubTemperatureAlertMultiplier, hubAccelTimeout, hubLoraPower, hubMovementHysteresisUp,
      hubMovementHysteresisDown, hubTemperatureAlertThresh, rawtime, HubSampleFrequency);
#else

#ifndef TEMPERATURE_LOGGING
    sprintf(hubCfgbuf, "%s,U,3,%s,%d,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", hid, Gateway.HubList[index].MeasId, Gateway.HubList[index].SensorID,
        Gateway.HubList[index].SampleCount, Gateway.HubList[index].SampleTime, Gateway.HubList[index].SlotNumber, Gateway.SamplePeriod,
        CurrentPeriodCount, Gateway.SlotCount, Gateway.HubList[index].LoraFreq, Gateway.HubList[index].SampleFreq,
        Gateway.HubList[index].HubFFT, Gateway.HubList[index].freqThresh, Gateway.HubList[index].accThresh);
#else
    sprintf(hubCfgbuf, "%s,U,3,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d\n", hid, Gateway.HubList[index].MeasId, Gateway.HubList[index].SensorID,
        Gateway.HubList[index].SampleCount, Gateway.HubList[index].SampleTime, Gateway.HubList[index].SlotNumber, Gateway.SamplePeriod,
        CurrentPeriodCount, Gateway.SlotCount, Gateway.HubList[index].LoraFreq, Gateway.HubList[index].SampleFreq,
        Gateway.HubList[index].HubFFT, Gateway.HubList[index].freqThresh, Gateway.HubList[index].accThresh, Gateway.HubList[index].temperatureSampleTime,
        Gateway.HubList[index].dfuBle, Gateway.HubList[index].FwVersion, Gateway.HubList[index].LoraPower, Gateway.HubList[index].CaptureMode);

     printf("Sending params to Hub: %s\n", hubCfgbuf);

#endif //TEMPERATURE_LOGGING
#endif //SIMPLE_TIME_UPDATE
    //printf("CC = %d --- PC = %d\n", currentCycle, testabc);
  }
}

bool checkForHOTA(char *hid) {
  int index = Gateway.HubCnt;
  int i;

  for (i = 0; i < Gateway.HubCnt; i++) {
    if (strcmp(Gateway.HubList[i].hubId, hid) == 0) {
      index = i;
      break;
    }
  }

  if(Gateway.HubList[index].dfuBle == 1) return true;

  return false;
}

void parse_hubInfo(char *buf, int hubSeq, bool initialRun) {
/*
    char temp_hnum;
    char temp_hid;
    int hcnt = 0;
    int i;
*/
    printf("Inside parse_hubInfo buf length = %u\n", strlen(buf));
    printf("str target = %s \n", buf);
    if (strcmp(buf,"false") == 0)
      return;
//    sleep(1);
    if (strlen(buf)) {
        char *pch;
        //-----------------------------------------
        // Extract Each Hubs data
        //-----------------------------------------

        //  while (pch != NULL) {
        //for (i = 0; i < Gateway.HubCnt; i++) {
        pch = strsep(&buf, ",");
        if (pch != NULL)
            if(initialRun)
              strcpy(Gateway.HubList[hubSeq].MeasId, pch + 1);
            else
              strcpy(Gateway.HubList[hubSeq].MeasId, pch);
        else
            strcpy(Gateway.HubList[hubSeq].MeasId, "");
	printf("MeasId in Parse HUB = %s \n", Gateway.HubList[hubSeq].MeasId);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            strcpy(Gateway.HubList[hubSeq].hubId, pch);
        else
            strcpy(Gateway.HubList[hubSeq].hubId, "");

        printf(" %d : pch = %s hubId = %s\n", hubSeq, pch, Gateway.HubList[hubSeq].hubId);


        //printf(" %d : pcgh = %s hubNo = %s\n", i, pch, Gateway.HubList[i].hubNo);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            //Gateway.HubList[hubSeq].MeasId = atoi(pch);
            strcpy(Gateway.HubList[hubSeq].hubNo, pch);
        else
            //Gateway.HubList[hubSeq].MeasId = 0;
            strcpy(Gateway.HubList[hubSeq].hubNo, "");
        //printf(" %d : pch = %s MeasId = \n%d",i, pch , Gateway.HubList[i].MeasId);
	printf("Hub Seq no in Parse HUB = %s \n", Gateway.HubList[hubSeq].hubNo);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].SensorID = atoi(pch);
        else
            Gateway.HubList[hubSeq].SensorID = 1;
	printf("Sensor ID no in Parse HUB = %d \n", Gateway.HubList[hubSeq].SensorID);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].SampleCount = atoi(pch);
        else
            Gateway.HubList[hubSeq].SampleCount = 0;
	printf("Sample Count  no in Parse HUB = %d \n", Gateway.HubList[hubSeq].SampleCount);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].SampleTime = atoi(pch);
        else
            Gateway.HubList[hubSeq].SampleTime = 0;
	printf("Sample Time  in Parse HUB = %d \n", Gateway.HubList[hubSeq].SampleTime);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].SlotNumber = atoi(pch);
        else
            Gateway.HubList[hubSeq].SlotNumber = 0;
	printf("Slot number in Parse HUB = %d \n", Gateway.HubList[hubSeq].SlotNumber);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].LoraFreq = atoi(pch);
        else
//            Gateway.HubList[hubSeq].LoraFreq = 915;
            Gateway.HubList[hubSeq].LoraFreq = 916;
	printf("Lora Frequency in Parse HUB = %d \n", Gateway.HubList[hubSeq].LoraFreq);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].SampleFreq = atoi(pch);
        else
            Gateway.HubList[hubSeq].SampleFreq = 1600;

	printf("====> Sample Frequency parse = %d\n",Gateway.HubList[hubSeq].SampleFreq );

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].HubFFT = atoi(pch);
        else
            Gateway.HubList[hubSeq].HubFFT = 0;
	printf("HUB FFT in Parse HUB = %d \n", Gateway.HubList[hubSeq].HubFFT);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].accThresh = atoi(pch);
        else
            Gateway.HubList[hubSeq].accThresh = 200;
	printf("Acc Threshold in Parse HUB = %d \n", Gateway.HubList[hubSeq].accThresh);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].freqThresh = atoi(pch);
        else
            Gateway.HubList[hubSeq].freqThresh = 100;
	printf("Frequency Threshold in Parse HUB = %d \n", Gateway.HubList[hubSeq].freqThresh);

#ifdef TEMPERATURE_LOGGING
        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].dfuBle = atoi(pch);
        else
            Gateway.HubList[hubSeq].dfuBle = 0;
	printf("DFU BLE Flagin Parse HUB = %d \n", Gateway.HubList[hubSeq].dfuBle);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.dfuGo = atoi(pch);
        else
            Gateway.dfuGo = 0;
	printf("DFU Go Flagin Parse HUB = %d \n", Gateway.dfuGo);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].temperatureSampleTime = atoi(pch);
        else
            Gateway.HubList[hubSeq].temperatureSampleTime = 0;
            //Gateway.dfuGo = 0;
	printf("Temp Sample Time Parse HUB = %d \n", Gateway.HubList[hubSeq].temperatureSampleTime);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            memset(Gateway.HubList[hubSeq].FwVersion,0,20);
            strcpy(Gateway.HubList[hubSeq].FwVersion, pch);
        } else {
            memset(Gateway.HubList[hubSeq].FwVersion,0,20);
            strcpy(Gateway.HubList[hubSeq].FwVersion, "0");
        }
	printf("FW version Parse HUB = %s \n", Gateway.HubList[hubSeq].FwVersion);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].LoraPower = atoi(pch);
        else
            Gateway.HubList[hubSeq].LoraPower = 0;

	printf(" Lora Power Parse HUB = %d \n", Gateway.HubList[hubSeq].LoraPower);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            Gateway.HubList[hubSeq].CaptureMode = atoi(pch);
        else
            Gateway.HubList[hubSeq].CaptureMode = 1;
	printf(" Capture Mode in Parse HUB = %d \n", Gateway.HubList[hubSeq].CaptureMode);
#endif //TEMPERATURE_LOGGING

#ifdef TEST
        printf("\nFinished Parsing Vehicle Info. Hub Number = %d\n", hubSeq);
#endif //TEST
    }
}


void parse_heartbeatInfo(char *buf) {

    printf("Inside parse_heartbeatInfo buf length = %u\n", strlen(buf));
    printf("str target = %s \n", buf);
    if (strcmp(buf,"false") == 0)
      return;

    if (strlen(buf)) {
        char *pch;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) > 0 && atoi(pch) < 86400) {
              heartbeatTimeout = atoi(pch);
            }
        } else {
            return;
        }

        printf("heartbeat Timeout set to = %d \n", heartbeatTimeout);

        pch = strsep(&buf, ",");
        if (pch != NULL)
            if(atoi(pch) > 0 && atoi(pch) < 86400) hubTimeout = atoi(pch);
        else
            return;

        printf("hub timeout set to = %d\n", hubTimeout);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) > 0 && atoi(pch) < 86400) hubTimeoutMulti = atoi(pch);
            printf("hubTimeoutMulti = %d\n", hubTimeoutMulti);
        }
        else
            return;

        printf("hub sleep multiplier set to = %d\n", hubTimeoutMulti);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 1) hubAccUpload = atoi(pch);
            printf("hubAccUpload = %d\n", hubAccUpload);
        }
        else
            return;

        printf("hub Acc Upload set to = %d\n", hubAccUpload);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) > 0 && atoi(pch) <= 3600) hubLineBusyBackoff = atoi(pch);
            printf("hubLineBusyBackoff = %d\n", hubLineBusyBackoff);
        }
        else
            return;

        printf("hub Line Busy Backoff set to = %d\n", hubLineBusyBackoff);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) > 0 && atoi(pch) <= 3600) hubRetryBackoff = atoi(pch);
            printf("hubRetryBackoff = %d\n", hubRetryBackoff);
        }
        else
            return;

        printf("hub Retry Backoff set to = %d\n", hubRetryBackoff);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 3600) hubTemperatureTimeout = atoi(pch);
            printf("hubTemperatureTimeout = %d\n", hubTemperatureTimeout);
        }
        else
            return;

        printf("hub Temperature Timeout set to = %d\n", hubTemperatureTimeout);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 15) hubSensorSelector = atoi(pch);
            printf("hubSensorSelector = %d\n", hubSensorSelector);
        }
        else
            return;

        printf("hub Sensor Selector set to = %d\n", hubSensorSelector);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 2 && atoi(pch) <= 4096) hubNumberOfSamples = atoi(pch);
            printf("hubNumberOfSamples = %d\n", hubNumberOfSamples);
        }
        else
            return;

        printf("hub number of samples set to = %d\n", hubNumberOfSamples);

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 64) hubMovementThreshold = atoi(pch);
            printf("hubMovementThreshold = %d\n", hubMovementThreshold);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 120) hubTemperatureMultiplier = atoi(pch);
            printf("hubTemperatureMultiplier = %d\n", hubTemperatureMultiplier);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 3600) hubAccelTimeout = atoi(pch);
            printf("hubAccelTimeout = %d\n", hubAccelTimeout);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 23) hubLoraPower = atoi(pch);
            printf("hubLoraPower = %d\n", hubLoraPower);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 20) hubMovementHysteresisUp = atoi(pch);
            printf("hubMovementHysteresisUp = %d\n", hubMovementHysteresisUp);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 20) hubMovementHysteresisDown = atoi(pch);
            printf("hubMovementHysteresisDown = %d\n", hubMovementHysteresisDown);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 3600) hubTemperatureAlertTimeout = atoi(pch);
            printf("hubTemperatureAlertTimeout = %d\n", hubTemperatureAlertTimeout);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 200) hubMovementReadThreshold = atoi(pch);
            printf("hubMovementReadThreshold = %d\n", hubMovementReadThreshold);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= 0 && atoi(pch) <= 120) hubTemperatureAlertMultiplier = atoi(pch);
            printf("hubTemperatureAlertMultiplier = %d\n", hubTemperatureAlertMultiplier);
        }
        else
            return;

        pch = strsep(&buf, ",");
        if (pch != NULL) {
            if(atoi(pch) >= -40 && atoi(pch) <= 125) hubTemperatureAlertThresh = atoi(pch);
            printf("hubTemperatureAlertThresh = %d\n", hubTemperatureAlertThresh);
        }
        else
            return;
    }
}

void print_gateway_info() {
    int i;
    printf("---------------------------------------\n");
    printf("Gateway Info                           \n");
    printf("---------------------------------------\n");
    printf("Gateway Vid = %s\n", Gateway.Vid);
    printf("Gateway Hub Count = %d\n", Gateway.HubCnt);
    for (i = 0; i < Gateway.HubCnt; i++) {
        printf("Hub # %d:  Id = %s   Lookup Id = %s SensorID = %d, Slot=%d, MeasId=%s, fft=%d, Sfrq=%d, AT=%d, FT=%d\n", i+1, Gateway.HubList[i].hubId,
            Gateway.HubList[i].hubNo, Gateway.HubList[i].SensorID, Gateway.HubList[i].SlotNumber,
            Gateway.HubList[i].MeasId, Gateway.HubList[i].HubFFT, Gateway.HubList[i].SampleFreq,
            Gateway.HubList[i].accThresh, Gateway.HubList[i].freqThresh);
//        sleep(1);
    }
}

bool SendTempDataFloats(int sampleCount, char *hid) {

  int sentCount = 0;
  int u = 1;

  //create_cell_connection();

  memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);
  memset(varbuf,0,CELL_VAR_MAX_LENGTH);
  sprintf(varbuf,"&l=1");

#ifdef QR_ID_UPLOADS
  sprintf(buf,"%s/addTempSet?hid=%s&HQR=%s&Gway=%s&GQR=%s&tcntpkt=%u&temparray=%d&ts=%s&seqno=%d", host_and_port, hid, config[hid].asCString(), Gateway.macId, config["GW_QR"].asCString(), 1000, 0, "1580496386", 10);
#else
  sprintf(buf,"%s/addTempSet?hid=%s&Gway=%s&tcntpkt=%u&temparray=%d&ts=%s&seqno=%d", host_and_port, hid, Gateway.macId, 1000, 0, "1580496386", 10);
#endif //QR_ID_UPLOADS

  //int lengthOfUplinkPacket = 480;
#ifdef TELIT
  int lengthOfUplinkPacket = 900;
#else
  int lengthOfUplinkPacket = 2000;
#endif //TELIT

  int headerLength = strlen(buf) + 2;

  int bytePerReading = 0;

  bytePerReading = 6;

  printf("bytePerReading = %d\n",bytePerReading);
  int readingsPerPacket = floor((float)(lengthOfUplinkPacket - headerLength) / (float)bytePerReading);
  printf("readingsPerPacket = %d\n",readingsPerPacket);
  int PacketsToSend = ceil((float)sampleCount / (float)readingsPerPacket);
  printf("PacketsToSend = %d\n",PacketsToSend);
puts("r1");


  while (sentCount < PacketsToSend) {

    memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);

    raw_time_get( &rawtime );

#ifdef QR_ID_UPLOADS
    sprintf(buf, "?hid=%s&HQR=%s&gway=%s&GQR=%s&ts=%d&seqno=%d&temparray=", hid, config[hid].asCString(), Gateway.macId, config["GW_QR"].asCString(), rawtime, sentCount);
#else
    sprintf(buf, "?hid=%s&gway=%s&ts=%ld&seqno=%d&temparray=", hid, Gateway.macId, rawtime, sentCount);
#endif //QR_ID_UPLOADS

    char temp[20];
    int numberOfSamplesInMessage = 0;
    for (int k = 0; k < readingsPerPacket; k++) {
      memset(temp, 0, 20);

      sprintf(temp, "%d,", loraBufTemperatureValues[(sentCount * readingsPerPacket) + k]);
      if(!(((sentCount * readingsPerPacket) + k) < sampleCount - 1)) {
        numberOfSamplesInMessage = k;
        k = readingsPerPacket;
      }

      strcat(buf, temp);
    }
puts("r2");

#ifdef CHARI
    buf[strlen(buf) - 1] = 0;
#endif // CHARI

    memset(temp,0,20);
    sprintf(temp,"&tcntpkt=%d", (numberOfSamplesInMessage+1));
    strcat(buf, temp);

    printf("Len of buf = %u\n", strlen(buf));
    printf("buf = %s\n", buf);

    //sentCount++;
    usleep(500000);

puts("r3");
    //gprs.httpConnect(HOST_AND_PORT "/addMeasData", buf);
    char postStr[2048];
    char respStr[512];
    memset(postStr,0,2048);
    memset(respStr,0,512);
    toJson(postStr, buf);

#ifdef DEBUG
    write_Log("Conmet_GW", "Sending hub temperature data", postStr, 0);
#endif

//add to nbuf
    err = nbuf_add(NBUF_CALL_POST, "/addTempSet", postStr, NULL);
    if (err)
        printf("Error: nbuf_add: %d\n", err);
//process nbuf
    nbuf_process();

    u++;
    sentCount++;
  }

  //AK printf("Data Sent : %s \n", buf);
  printf("Data Sent : no of packets send =  %d\n", sentCount);
  printf("\n");
  sleep(1);

  return true;
}

bool SendMeasDataFloatsMqtt(int sampleCount, char *measID, char *hid) {

  int sentCount = 0;
  int u = 1;
  int axisCounter = -1;

  //create_cell_connection();

  char mqttStr[40000];


  memset(mqttStr, 0, 40000);
  memset(varbuf,0,CELL_VAR_MAX_LENGTH);
  sprintf(varbuf,"&l=1");
  raw_time_get(&rawtime);

  char idStr[60];
  memset(idStr,0,60);

#ifdef QR_ID_UPLOADS
  sprintf(idStr, "%s&HQR=%s&GQR=%s", hid, config[hid].asCString(), config["GW_QR"].asCString());
#else
  sprintf(idStr, "%s", hid);
#endif //QR_ID_UPLOADS

#ifdef INT16_BASED_READINGS
  sprintf(mqttStr,"%s/addMeasData?hid=%s&mid=%s&seqno=%u&scalefactor=%f&md=&rdct=%d&ts=%ld", host_and_port, idStr, measID, 100, scaleFactor, 1000, rawtime);
#else
  sprintf(mqttStr,"%s/addMeasData?hid=%s&mid=%s&seqno=%u&md=&rdct=%d&ts=%d", host_and_port, idStr, measID, 100, 1000, rawtime);
#endif //INT16_BASED_READINGS


  //int lengthOfUplinkPacket = 480;
#ifdef TELIT
  int lengthOfUplinkPacket = 900;
#else
  int lengthOfUplinkPacket = 40000;
#endif //TELIT

  //printf("lengthOfUplinkPacket = %d\n",lengthOfUplinkPacket);
  int headerLength = strlen(mqttStr) + 2;
  //printf("headerLength = %d\n",headerLength);

  int bytePerReading = 0;

#ifdef CHARI
  bytePerReading = 8;
#endif // CHARI

#ifdef HEXI
  bytePerReading = 9;
#endif //HEXI

  //printf("bytePerReading = %d\n",bytePerReading);
  int readingsPerPacket = floor((float)(lengthOfUplinkPacket - headerLength) / (float)bytePerReading);
  //printf("readingsPerPacket = %d\n",readingsPerPacket);
  int PacketsToSend = ceil((float)sampleCount / (float)readingsPerPacket);
  //printf("PacketsToSend = %d\n",PacketsToSend);
puts("u1");


    memset(idStr,0,60);

#ifdef QR_ID_UPLOADS
    sprintf(idStr, "%s&HQR=%s&GQR=%s", hid, config[hid].asCString(), config["GW_QR"].asCString());
#else
    sprintf(idStr, "%s", hid);
#endif //QR_ID_UPLOADS

    memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);

    //sprintf(buf, "/addMeasData?mid=%s&seqno=%u&ax=%s&md=", measID, sentCount, "Z");
#ifdef MULTI_AXIS


#ifdef INT16_BASED_READINGS
        sprintf(mqttStr, "?hid=%s&mid=%s&seqno=%u&scalefactor=%f&md=", idStr, measID, sentCount, scaleFactor);
#else
        sprintf(mqttStr, "?hid=%s&mid=%s&seqno=%u&md=", idStr, measID, sentCount);
#endif //INT16_BASED_READINGS


#else //MULTI_AXIS

#ifdef INT16_BASED_READINGS
        sprintf(mqttStr, "?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=", idStr, measID, sentCount, "Z", scaleFactor);
#else
        sprintf(mqttStr, "?hid=%s&mid=%s&seqno=%u&ax=%s&md=", idStr, measID, sentCount, "Z");
#endif //INT16_BASED_READINGS

    axisCounter = 0;

#endif //MULTI_AXIS

    char temp[50];
    int numberOfSamplesInMessage = 0;
    for (int k = 0; k < sampleCount; k++) {
      memset(temp, 0, 50);

//#ifdef CHARI
#ifdef MULTI_AXIS

#ifdef INT16_BASED_READINGS
        sprintf(temp, "%d,%d,%d,", (int)(loraBufValues[k]/scaleFactor), (int)(loraBufValues[k+sampleCount]/scaleFactor), (int)(loraBufValues[k+(2*sampleCount)]/scaleFactor));
#else
        sprintf(temp, "%2.3f,%2.3f,%2.3f,", loraBufValues[k], loraBufValues[k+sampleCount], loraBufValues[k+(2*sampleCount)]);
#endif //INT16_BASED_READINGS

#else //MULTI_AXIS

#ifdef INT16_BASED_READINGS
        //printf("%d\n", ((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter));
        //usleep(10000);
        sprintf(temp, "%d,", (int)(loraBufValues[k]/scaleFactor));
#else
        sprintf(temp, "%2.3f,", loraBufValues[k]);
#endif //INT16_BASED_READINGS

      //sprintf(temp, "%2.2f,", loraBufValues[(sentCount * readingsPerPacket) + k]);
      //if(!(((sentCount * readingsPerPacket) + k) < sampleCount - 1)) {
      //  numberOfSamplesInMessage = k;
      //  k = readingsPerPacket;
      //}
#endif //MULTI_AXIS
      //if(loraBufValues[(sentCount * readingsPerPacket) + k] != 0) printf("non zero value = %f\n", loraBufValues[(sentCount * readingsPerPacket) + k]);
//#endif //CHARI

/*#ifdef HEXI
      float tempF = loraBufValues[(sentCount * readingsPerPacket) + k];
      int16_t tempI = tempF * 100;
      uint8_t partA = (tempI >> 8);
      uint8_t partB = tempI;
      //printf("%02X%02X\n", partA, partB);
      sprintf(temp, "%02X%02X", partA, partB);
#endif //HEXI*/

      strcat(mqttStr, temp);
    }
puts("u2");

    //if(sentCount == (PacketsToSend-1)) strcat(buf,varbuf);

#ifdef CHARI
    buf[strlen(mqttStr) - 1] = 0;
#endif // CHARI

    memset(temp,0,20);
    sprintf(temp,"&rdct=%d&ts=%ld", sampleCount*3, rawtime);

    strcat(mqttStr, temp);

    printf("Len of buf = %u\n", strlen(mqttStr));
    printf("buf = %s\n", mqttStr);

    //sentCount++;
    usleep(500000);

puts("u3");
    //gprs.httpConnect(HOST_AND_PORT "/addMeasData", buf);
    char postStr[50000];
    char respStr[512];
    memset(postStr,0,50000);
    memset(respStr,0,512);
    toJson(postStr, mqttStr);

#ifdef DEBUG
    //write_Log("Conmet_GW", "Sending hub accel data", postStr, 0);
#endif

//add to nbuf
    err = nbuf_add(NBUF_CALL_MQTT, "/addMeasDataXYZ", postStr, NULL);
    if (err)
        printf("Error: nbuf_add: %d\n", err);
//process nbuf
    nbuf_process();

    u++;
    sentCount++;
//#ifdef MULTI_AXIS
//  }
//#endif //MULTI_AXIS
  //}

  //AK printf("Data Sent : %s \n", buf);
  printf("Data Sent : no of packets send =  %d\n", sentCount);
  printf("\n");
  sleep(1);

  return true;
}

bool SendMeasDataFloats(int sampleCount, char *measID, char *hid) {

  int sentCount = 0;
  int u = 1;
  int axisCounter = -1;

  //create_cell_connection();

  memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);
  memset(varbuf,0,CELL_VAR_MAX_LENGTH);
  sprintf(varbuf,"&l=1");
  raw_time_get(&rawtime);

  char idStr[60];
  memset(idStr,0,60);

#ifdef QR_ID_UPLOADS
  sprintf(idStr, "%s&HQR=%s&GQR=%s", hid, config[hid].asCString(), config["GW_QR"].asCString());
#else
  sprintf(idStr, "%s", hid);
#endif //QR_ID_UPLOADS

#ifdef INT16_BASED_READINGS
  sprintf(buf,"%s/addMeasData?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=&rdct=%d&ts=%ld", host_and_port, idStr, measID, 100, "Z", scaleFactor, 1000, rawtime);
#else
  sprintf(buf,"%s/addMeasData?hid=%s&mid=%s&seqno=%u&ax=%s&md=&rdct=%d&ts=%d", host_and_port, idStr, measID, 100, "Z", 1000, rawtime);
#endif //INT16_BASED_READINGS


  //int lengthOfUplinkPacket = 480;
#ifdef TELIT
  int lengthOfUplinkPacket = 900;
#else
  int lengthOfUplinkPacket = 2000;
#endif //TELIT

  //printf("lengthOfUplinkPacket = %d\n",lengthOfUplinkPacket);
  int headerLength = strlen(buf) + 2;
  //printf("headerLength = %d\n",headerLength);

  int bytePerReading = 0;

#ifdef CHARI
  bytePerReading = 8;
#endif // CHARI

#ifdef HEXI
  bytePerReading = 9;
#endif //HEXI

  //printf("bytePerReading = %d\n",bytePerReading);
  int readingsPerPacket = floor((float)(lengthOfUplinkPacket - headerLength) / (float)bytePerReading);
  //printf("readingsPerPacket = %d\n",readingsPerPacket);
  int PacketsToSend = ceil((float)sampleCount / (float)readingsPerPacket);
  //printf("PacketsToSend = %d\n",PacketsToSend);
puts("u1");

#ifdef MULTI_AXIS
  while (sentCount < (PacketsToSend*3)) {
#else //MULTI_AXIS
  while (sentCount < PacketsToSend) {
#endif //MULTI_AXIS

    memset(idStr,0,60);

#ifdef QR_ID_UPLOADS
    sprintf(idStr, "%s&HQR=%s&GQR=%s", hid, config[hid].asCString(), config["GW_QR"].asCString());
#else
    sprintf(idStr, "%s", hid);
#endif //QR_ID_UPLOADS

    memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);

    //sprintf(buf, "/addMeasData?mid=%s&seqno=%u&ax=%s&md=", measID, sentCount, "Z");
#ifdef MULTI_AXIS
    if((sentCount % PacketsToSend) == 0) {
      axisCounter++;
    }
    switch(axisCounter) {
      case 0:
#ifdef INT16_BASED_READINGS
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=", idStr, measID, sentCount, "Z", scaleFactor);
#else
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&md=", idStr, measID, sentCount, "Z");
#endif //INT16_BASED_READINGS
        break;

      case 1:
#ifdef INT16_BASED_READINGS
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=", idStr, measID, sentCount, "X", scaleFactor);
#else
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&md=", idStr, measID, sentCount, "X");
#endif //INT16_BASED_READINGS
        break;

      case 2:
#ifdef INT16_BASED_READINGS
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=", idStr, measID, sentCount, "Y", scaleFactor);
#else
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&md=", idStr, measID, sentCount, "Y");
#endif //INT16_BASED_READINGS
        break;
    }
#else //MULTI_AXIS

#ifdef INT16_BASED_READINGS
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&scalefactor=%f&md=", idStr, measID, sentCount, "Z", scaleFactor);
#else
        sprintf(buf, "?hid=%s&mid=%s&seqno=%u&ax=%s&md=", idStr, measID, sentCount, "Z");
#endif //INT16_BASED_READINGS

    axisCounter = 0;

#endif //MULTI_AXIS

    char temp[20];
    int numberOfSamplesInMessage = 0;
    for (int k = 0; k < readingsPerPacket; k++) {
      memset(temp, 0, 20);

//#ifdef CHARI
#ifdef MULTI_AXIS

#ifdef INT16_BASED_READINGS
        sprintf(temp, "%d,", (int)(loraBufValues[((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter)]/scaleFactor));
#else
        sprintf(temp, "%2.3f,", loraBufValues[((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter)]);
#endif //INT16_BASED_READINGS

      //sprintf(temp, "%2.2f,", loraBufValues[(sentCount * readingsPerPacket) + k]);
        if(!((((sentCount-(axisCounter*PacketsToSend)) * readingsPerPacket) + k) < sampleCount - 1)) {
          numberOfSamplesInMessage = k;
          k = readingsPerPacket;
        }

#else //MULTI_AXIS

#ifdef INT16_BASED_READINGS
        //printf("%d\n", ((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter));
        //usleep(10000);
        sprintf(temp, "%d,", (int)(loraBufValues[((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter)]/scaleFactor));
#else
        sprintf(temp, "%2.3f,", loraBufValues[((sentCount-(PacketsToSend*axisCounter)) * readingsPerPacket) + k + (sampleCount*axisCounter)]);
#endif //INT16_BASED_READINGS

      //sprintf(temp, "%2.2f,", loraBufValues[(sentCount * readingsPerPacket) + k]);
      if(!(((sentCount * readingsPerPacket) + k) < sampleCount - 1)) {
        numberOfSamplesInMessage = k;
        k = readingsPerPacket;
      }
#endif //MULTI_AXIS
      //if(loraBufValues[(sentCount * readingsPerPacket) + k] != 0) printf("non zero value = %f\n", loraBufValues[(sentCount * readingsPerPacket) + k]);
//#endif //CHARI

/*#ifdef HEXI
      float tempF = loraBufValues[(sentCount * readingsPerPacket) + k];
      int16_t tempI = tempF * 100;
      uint8_t partA = (tempI >> 8);
      uint8_t partB = tempI;
      //printf("%02X%02X\n", partA, partB);
      sprintf(temp, "%02X%02X", partA, partB);
#endif //HEXI*/

      strcat(buf, temp);
    }
puts("u2");

    //if(sentCount == (PacketsToSend-1)) strcat(buf,varbuf);

#ifdef CHARI
    buf[strlen(buf) - 1] = 0;
#endif // CHARI

    memset(temp,0,20);
    if(numberOfSamplesInMessage == 0)
      sprintf(temp,"&rdct=%d&ts=%ld", (readingsPerPacket), rawtime);
    else
      sprintf(temp,"&rdct=%d&ts=%ld", (numberOfSamplesInMessage+1), rawtime);

    strcat(buf, temp);

    printf("Len of buf = %u\n", strlen(buf));
    printf("buf = %s\n", buf);

    //sentCount++;
    usleep(500000);

puts("u3");
    //gprs.httpConnect(HOST_AND_PORT "/addMeasData", buf);
    char postStr[2048];
    char respStr[512];
    memset(postStr,0,2048);
    memset(respStr,0,512);
    toJson(postStr, buf);

#ifdef DEBUG
    write_Log("Conmet_GW", "Sending hub accel data", postStr, 0);
#endif

//add to nbuf
    err = nbuf_add(NBUF_CALL_POST, "/addMeasDatapost", postStr, NULL);
    if (err)
        printf("Error: nbuf_add: %d\n", err);
//process nbuf
    nbuf_process();

    u++;
    sentCount++;
//#ifdef MULTI_AXIS
//  }
//#endif //MULTI_AXIS
  }

  //AK printf("Data Sent : %s \n", buf);
  printf("Data Sent : no of packets send =  %d\n", sentCount);
  printf("\n");
  sleep(1);

  return true;
}

bool send_acceldata_to_cloud(char *hid, int pktCount) {

  printf("Sending Accel Data to Cloud\n");

  int nodeID;
  char msgType[2];
  char sensorIDStr[6];
  char loraBufTemp[255];
  int sensorID;
  int someNo = 0;
  float temp1;
  float temp2;
  float temp3;
  int accThresh;
  int freqThresh;
  int freqMax;
  int accMax;
  int tempAvg;
  int tempMax;
  int tempMaxCount;
  int tempMin;
  int tempMinCount;
  int loraPower;
  int captureMode;

  float bat;
  int sampleDuration;
  uint32_t utcTimeNode;
  int sampleCnt;
  char *pch;
  char MeasID[20];
  char FwVer[20];

  //-----------------------------------------------
  // Extract Measurement Meta data
  //-----------------------------------------------
  memset(loraBufTemp,0,255);
  memcpy(loraBufTemp, lora_buf[0]+4, 251);

  //pch = strtok((char *)lora_buf[0], ",");
  pch = strtok(loraBufTemp, ",");
  if(pch == NULL) return false;
  nodeID = atoi(pch);

  pch = strtok(NULL, ",");
  printf("pch = %s\n", pch);
  strcpy(msgType, pch);

  pch = strtok(NULL, ",");
  sensorID = atoi(pch);

  pch = strtok(NULL, ",");
  temp1 = atof(pch);

  pch = strtok(NULL, ",");
  temp2 = atof(pch);

  pch = strtok(NULL, ",");
  temp3 = atof(pch);

  pch = strtok(NULL, ",");
  bat = atof(pch);

  pch = strtok(NULL, ",");
  sampleCnt = atoi(pch);

  pch = strtok(NULL, ",");
  sampleDuration = atoi(pch);

  pch = strtok(NULL, ",|");
  strcpy(MeasID, pch);

  pch = strtok(NULL, ",");
  char timeInHex[9];
  uint32_t num;
  strcpy(timeInHex, pch);
  sscanf(timeInHex, "%x", &num);
  utcTimeNode = *((uint32_t*)&num);
  //printf("utcTime = %d\n", utcTimeNode);

#ifdef INT16_BASED_READINGS
  pch = strtok(NULL, ",");
  bitsPerMeasurement = atoi(pch);

  pch = strtok(NULL, ",|");
  char floatInHex[9];
  num = 0;
  strcpy(floatInHex, pch);
  sscanf(floatInHex, "%x", &num);
  scaleFactor = *((float*)&num);
  //printf("scalefactor = %f\n", scaleFactor);
#endif //INT16_BASED_READINGS

  pch = strtok(NULL, ",");
  if(pch != NULL) {
puts("st0.7");
#ifndef SIMPLE_TIME_UPDATE
    tempAvg = atoi(pch);

    pch = strtok(NULL, ",");
    tempMax = atoi(pch);

    pch = strtok(NULL, ",");
    tempMaxCount = atoi(pch);

    pch = strtok(NULL, ",");
    tempMin = atoi(pch);

    pch = strtok(NULL, ",|");
    tempMinCount = atoi(pch);

    pch = strtok(NULL, ",");
    memset(FwVer,0,20);
    strcpy(FwVer,pch);

    pch = strtok(NULL, ",");
    loraPower = atoi(pch);

    pch = strtok(NULL, ",");
    captureMode = atoi(pch);

    pch = strtok(NULL, ",");
    if(pch != NULL) {
puts("st0.8");
      freqThresh = atoi(pch);

      pch = strtok(NULL, ",");
      accThresh = atoi(pch);

      pch = strtok(NULL, ",");
      freqMax = atoi(pch);

      pch = strtok(NULL, "|");
      accMax = atoi(pch);
    } else {
      freqThresh = 0;
      accThresh = 0;
      freqMax = 0;
      accMax = 0;
    }
#else
      freqThresh = atoi(pch);

      pch = strtok(NULL, ",");
      accThresh = atoi(pch);

      pch = strtok(NULL, ",");
      freqMax = atoi(pch);

      pch = strtok(NULL, "|");
      accMax = atoi(pch);
#endif //SIMPLE_TIME_UPDATE
puts("st0.9");
  } else {
    freqThresh = 0;
    accThresh = 0;
    freqMax = 0;
    accMax = 0;
    tempAvg = 0;
    tempMax = 0;
    tempMaxCount = 0;
    tempMin = 0;
    tempMinCount = 0;
    loraPower = 10;
    captureMode = 1;
  }
puts("st1");

  printf("-----------------------------------\n");
  printf("packet Data\n");
  printf("nodeID = %d\n", nodeID);
  printf("Message Type = %s\n", msgType);
  printf("sensorID = %d\n", sensorID);
  sprintf(sensorIDStr, "%d", sensorID);
  printf("some No = %d\n", someNo);
  printf("temp1 = %f\n", temp1);
  printf("temp2 = %f\n", temp2);
  printf("temp3 = %f\n", temp3);
  printf("bat = %f\n", bat);
  printf("SampleCnt= %d\n", sampleCnt);
  printf("sampleTime = %d\n", sampleDuration);
  printf("Measurement ID = %s\n", MeasID);
  printf("freqThesh= %d\n", freqThresh);
  printf("accThresh = %d\n", accThresh);
  printf("freqMax = %d\n", freqMax);
  printf("accMax = %d\n", accMax);

#ifdef TEMPERATURE_LOGGING
  printf("tempAvg = %d\n", tempAvg);
  printf("tempMax = %d\n", tempMax);
  printf("tempMin = %d\n", tempMin);
  printf("tempMaxCount = %d\n", tempMaxCount);
  printf("tempMinCount = %d\n", tempMinCount);
  printf("Firmware Version = %s\n", FwVer);
  printf("Lora transmit power = %d\n", loraPower);
  printf("Capture Mode = %d\n", captureMode);
#endif
  printf("-----------------------------------\n");

  if(pktCount != 1) loraBufDecode();

puts("st1.33");
  printf("pkt = %d\n", pktCount);
#ifdef TEMPERATURE_DECODING
  if(pktCount >= 2) SendTempDataFloats(loraValTemperatureCounter, hid);
  if(pktCount >= 3) SendMeasDataFloats(sampleCnt, MeasID, hid);
#ifdef MQTT
  if(pktCount >= 3) SendMeasDataFloatsMqtt(sampleCnt, MeasID, hid);
#endif //MQTT
#else //TEMPERATURE_DECODING
  if(pktCount >= 2) SendMeasDataFloats(sampleCnt, MeasID, hid);
#ifdef MQTT
  if(pktCount >= 2) SendMeasDataFloatsMqtt(sampleCnt, MeasID, hid);
#endif //MQTT
#endif //TEMPERATURE_DECODING

#ifdef GPS_TIMING
  //rtc_tm_setup_gps();
#endif //GPS_TIMING
puts("st1.66");
  raw_time_get( &rawtime );
puts("st2");

  memset(buf, 0, CELL_SEND_BUF_MAX_LENGTH);

  char idStrGway[50];
  char idStrHub[50];
  memset(idStrGway,0,50);
  memset(idStrHub,0,50);
#ifdef QR_ID_UPLOADS //config["GW_QR"].asCString()
  sprintf(idStrGway, "%s&GQR=%s", Gateway.macId, config["GW_QR"].asCString());
  sprintf(idStrHub, "%s&HQR=%s", hid, config[hid].asCString());
#else
  sprintf(idStrGway, "%s", Gateway.macId);
  sprintf(idStrHub, "%s", hid);
#endif //QR_ID_UPLOADS

#ifndef TEMPERATURE_LOGGING
  if(pktCount != 1) {
    sprintf(buf, "?vid=%s&hid=%s&sid=%s&mid=%s&%s&temp1=%.2f&temp2=%.2f&temp3=%.2f&bat=%.2f&stime=%d&scount=%d&gway=%s&lat=%.8f&lon=%.8f&aval=%d&fval=%d&hubat=%d&hubft=%d&l=1",
      Gateway.Vid, idStrHub, sensorIDStr, MeasID, timeString, temp1, temp2, temp3, bat, sampleDuration,
      sampleCnt, idStrGway, Gateway.gps_data.lat, Gateway.gps_data.lon, accMax, freqMax, accThresh, freqThresh);
puts("st3");
  } else {
    sprintf(buf, "?vid=%s&hid=%s&sid=%s&mid=%s&%s&temp1=%.2f&temp2=%.2f&temp3=%.2f&bat=%.2f&stime=%d&scount=%d&gway=%s&lat=%.8f&lon=%.8f&aval=%d&fval=%d&hubat=%d&hubft=%d&l=1",
      Gateway.Vid, idStrHub, sensorIDStr, MeasID, timeString, temp1, temp2, temp3, bat, sampleDuration,
      sampleCnt, idStrGway, Gateway.gps_data.lat, Gateway.gps_data.lon, accMax, freqMax, accThresh, freqThresh);
puts("st4");
  }
#else
  if(pktCount != 1) {
//    sprintf(buf, "?vid=%s&hid=%s&sid=%s&mid=%s&%s&temp1=%.2f&temp2=%.2f&temp3=%.2f&bat=%.2f&stime=%d&scount=%d&gway=%s&lat=%.8f&lon=%.8f&aval=%d&fval=%d&hubat=%d&hubft=%d&l=1&minTemp=%d&minTempT=%d&maxTemp=%d&maxTempT=%d&avgTemp=%d&fmv=%s&tpwrth=%d&capturemode=%d&rssi=%d",
//      Gateway.Vid, idStrHub, sensorIDStr, MeasID, timeString, temp1, temp2, temp3, bat, sampleDuration,
//      sampleCnt, idStrGway, Gateway.gps_data.lat, Gateway.gps_data.lon, accMax, freqMax, accThresh, freqThresh,
//      tempMin, tempMinCount, tempMax, tempMaxCount, tempAvg, FwVer, loraPower, captureMode, lastRssiLora);

    sprintf(buf, "?vid=%s&hid=%s&sid=%s&mid=%s&ts=%ld&temp1=%.2f&temp2=%.2f&temp3=%.2f&bat=%.2f&stime=%d&scount=%d&gway=%s&lat=%.8f&lon=%.8f&l=1&minTemp=%d&minTempT=%d&maxTemp=%d&maxTempT=%d&avgTemp=%d&tpwrth=%d&capturemode=%d&rssi=%d",
      Gateway.Vid, idStrHub, sensorIDStr, MeasID, rawtime, temp1, temp2, temp3, bat, sampleDuration,
      sampleCnt, idStrGway, Gateway.gps_data.lat, Gateway.gps_data.lon,
      tempMin, tempMinCount, tempMax, tempMaxCount, tempAvg, loraPower, captureMode, lastRssiLora);
puts("st3");
  } else {
    sprintf(buf, "?vid=%s&hid=%s&sid=%s&mid=%s&ts=%ld&temp1=%.2f&temp2=%.2f&temp3=%.2f&bat=%.2f&stime=%d&scount=%d&gway=%s&lat=%.8f&lon=%.8f&aval=%d&fval=%d&hubat=%d&hubft=%d&l=1&minTemp=%d&minTempT=%d&maxTemp=%d&maxTempT=%d&avgTemp=%d&fmv=%s&tpwrth=%d&capturemode=%d&rssi=%d",
      Gateway.Vid, idStrHub, sensorIDStr, MeasID, rawtime, temp1, temp2, temp3, bat, sampleDuration,
      sampleCnt, idStrGway, Gateway.gps_data.lat, Gateway.gps_data.lon, accMax, freqMax, accThresh, freqThresh,
      tempMin, tempMinCount, tempMax, tempMaxCount, tempAvg, FwVer, loraPower, captureMode, lastRssiLora);
puts("st4");
  }
#endif //TEMPERATURE_LOGGING

  /*setupTCA();

  if (!sim_init()) {
    printf("sim init failed\n");
    return false;
  }

  bool locflag = false;
  //---------------------------------------
  // create http connection
  //---------------------------------------
  if (!httpInit()) {
    printf("HTTP init failed\n");
    return false;
  }*/

  //---------------------------------------
  // Get the GPS location from Cell Modem
  //---------------------------------------

  /*
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
    if (gprs.getLocation(&loc)) {
      locflag = true;
      break;
    }
    }

    */
#ifdef CELL_TIMING
    rtc_tm_setup_gprs();
#endif //CELL_TIMING

  //--------------------------------------------
  // Prepape buffer for upload
  //--------------------------------------------

  //  sprintf(buf, "?id=%u&subID=%u&subLog=%d&MUR=%2.2f&STS=%2.2f&TMP=%2.2f&ts=20%d-%02d-%02dT%02d:%02d:%02d&bat=%.3f&SAMP=%d&TIM=%d",
  //    id, SUB_ID, SUB_LOG_NUM, MUR_TEMP, STS_TEMP, TMP_TEMP, SUB_BAT, TIMESTAMP, BAT, SUB_SAMPLES, SUB_TIME);

  //gprs.httpConnect(HOST_AND_PORT "/addMeasReadingCfg", buf);
  //gprs.httpConnect(HOST_AND_PORT, buf);

/*
  start_timer(60000);
  while (!timerFlag) {
    checkhttp = gprs.httpIsConnected();
    if (checkhttp == 1)
      break;
  }

  if (timerFlag) {
    printf("httpIsConnected Failed\n");
    systemSleep();
    //   disable_sim();
    return false;
  }
  stop_timer();
*/
  //post
  ///addMeasReadingCfg
  char postStr[2048];
  char respStr[512];
  printf("%s\n", buf);
  toJson(postStr, buf);
  printf("Data to be send\n");
  printf("%s\n", postStr);

#ifdef DEBUG
  write_Log("Conmet_GW", "Sending hub data", postStr, 0);
#endif

//add to nbuf
    err = nbuf_add(NBUF_CALL_POST, "/addMeasReadingCfgpost", postStr, NULL);
    if (err)
        printf("Error: nbuf_add: %d\n", err);
//process nbuf
    nbuf_process();

  return true;
}

void lora_receiver(void) {



	#ifdef TEST
  printf("Lora Scan -- Starting\n");
#endif


  counter = 0;

  int endFlag = 0;
  int startFlag = 0;
  uint8_t tokbuf[255];
  char logStr[512];
  char hid[20];
  char curId[20];
  //int currentID = 0;
  char testId[20];
  char cmd[10];
  int curSeqNum = -1;
  int lastSeqNum = -1;
  //int hid_set = 0;

  memset(tokbuf, 0, 255);

  uint8_t networkHeader[4] = {0x7F, 0xFF, 0xFF, 0x00};

 //stop_timer();

  //while (!endFlag && !timerFlag) {
  while (!endFlag) {

	int state = readRegister(0x01);
	printf("state = %d\n", state);
	int respondAFlag = 0;
    int respondUFlag = 0;
    //uint32_t testid = 0;
    //memset(measData, 0, 12000);
    raw_time_get( &rawtime );

    if((rawtime - prelogtime > heartbeatTimeout) && !startFlag) {
        puts("rawtime - prelogtime > LORA_HEARTBEAT_TIMEOUT) && !startFlag");
#ifdef SIMPLE_TIME_UPDATE
        heartbeatHit = true;
#endif //SIMPLE_TIME_UPDATE
        return;
     }

    if((rawtime - lastloratime > LORA_DATA_TIMEOUT) && startFlag) return;

    //if(digitalRead(LORA_DEFAULT_DIO0_PIN)) {
    //  rf95.clearRxIrq();
    //}

    int testPrev , testCur = 0;
    raw_time_get( &time_c );
    testCur = currentCycle;
    testPrev = testabc;
    currentCycle = floor((time_c - time_o)/Gateway.SamplePeriod);
    if((((rawtime - lastrawtime) > Gateway.gps_timeout) || (currentCycle > testabc)) && !startFlag) {
      //printf("CurrentCycleTest = %d\nPrecious cycle test = %d\nCurrentCycle = %d\nPrecious cycle = %d\ntimec = %d\ntimeo = %d\n", testCur, testPrev, currentCycle, testabc, time_c, time_o);
      lastrawtime = rawtime;
//      pingGps();
      testabc = currentCycle;
    }

    if (rf95.avail()) {
puts("t1");
      memset(lora_buf[counter], 0, RH_RF95_MAX_MESSAGE_LEN);
      uint8_t len = sizeof(buff);
      if (rf95.recv(lora_buf[counter], &len)) {
puts("t2");
        //rf95.setModeRx();
        //printf("lora_buf: %2X %2X %2X %2X \n", lora_buf[counter][0], lora_buf[counter][1], lora_buf[counter][2], lora_buf[counter][3]);
        if(memcmp(networkHeader, lora_buf[counter], 4) == 0) {
        printf("\n");
        //printf("%d %d %d %s", counter, len, rf95.packetRssi(), lora_buf[counter]+4);
        int receivedRssi = 0;
        receivedRssi = rf95.packetRssi();
        lastSNRLora = rf95.packetSnr();
        lora_buf_count[counter] = len;
        printf("=====> The Last RSSI value = %d\n", receivedRssi);
        printf("=====> The Last SNR value = %d\n", lastSNRLora);
        sprintf(logStr, "%d %d %d %s", counter, len, rf95.packetRssi(), lora_buf[counter]+4);
        printf("%s\n", logStr);
        strcpy((char *)tokbuf, (char *)lora_buf[counter]+4);

#ifdef DEBUG
        write_Log("Conmet_GW", "Receiving lora message", logStr, 0);
#endif
puts("t3");

        char *pch;
        pch = strtok((char *)tokbuf, ",");

        if (pch != NULL) {
          //testid = atoi(pch);
          strcpy(testId, pch);
        }
        pch = strtok(NULL, ",");
        strcpy(cmd, pch);
        //strcpy(seq, pch);
puts("t3.5");
        char *cmd_ch;
        cmd_ch = strstr(cmd, "|");
        if(cmd_ch != NULL)
          strncpy(cmd_ch, "", 1);
//        respondAFlag = 1;
// strcpy(hid,"DD5D0CEDE6E9");
puts("t4");

        if (pch != NULL) {

          if ((strcmp(cmd, dataMessage) == 0) && (strcmp(testId, curId) == 0)) {
            startFlag = 1;
            raw_time_get( &lastloratime );
            printf("%d %d %s", counter, len, lora_buf[counter]+4);
            printf("HID = %s  curId = %s\n", hid, curId);
            respondAFlag = 1;
            pch = strtok(NULL, ",");
            curSeqNum = atoi(pch);
            if (curSeqNum != lastSeqNum)
              counter++;
            lastSeqNum = curSeqNum;
            lastRssiLoraSum += receivedRssi;
            lastRssiLoraCount++;
            //start_timer(20000);
puts("t5");
#ifdef SIMPLE_TIME_UPDATE
#ifdef CONFIG_HUB_WHITELIST
          } else if ((strcmp(cmd, dataStartMessage) == 0) && !startFlag && (checkHubId(testId) != -1)) {
#else
          } else if ((strcmp(cmd, dataStartMessage) == 0) && !startFlag) {
#endif //CONFIG_HUB_WHITELIST
#else
          } else if ((strcmp(cmd, dataStartMessage) == 0) && !startFlag && (checkHubId(testId) != -1)) {
#endif //SIMPLE_TIME_UPDATE
            if(!startFlag) raw_time_get( &lastloratime );
              strcpy(hid, testId);
              //strncpy(hid + strlen(hid) - 1, "", 1);
              //hid_set = 1;
#ifdef SIMPLE_TIME_UPDATE
//              getGps();
              err = gps_get(&Gateway.gps_data);
              if (err) {
                  printf("Error: gps: %d\n", err);
                  gps_invalid(&Gateway.gps_data);
//todo: this is only workaround for gps fix quality
                  gps_fix = 0;
              }
              else {
                  gps_fix = gps_fix_get(Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.satellites);
              }
//todo: this is only workaround for gps fix quality

#endif //SIMPLE_TIME_UPDATE
              memset(curId,0,20);
              strncpy(curId, hid + 10, 2);

              BleOtaFlag = false;
              BleOtaFlag = checkForHOTA(hid);

              lastRssiLoraSum += receivedRssi;
              lastRssiLoraCount++;

              printf("%d %d %s", counter, len, lora_buf[counter]+4);
              printf("HID = %s  curId = %s\n", hid, curId);
              if (counter == 0)
                counter++;
              //currentID = testid;
              respondAFlag = 1;
puts("t6");
          } else if ((strcmp(cmd, dataEndMessage) == 0) && (strcmp(testId, hid) == 0)) {
            printf("%d %d %s", counter, len, lora_buf[counter]+4);
            printf("HID = %s  curId = %s\n", hid, curId);
            lastRssiLoraSum += receivedRssi;
            lastRssiLoraCount++;

            lastRssiLora = lastRssiLoraSum/lastRssiLoraCount;
            lastRssiLoraCount = 0;
            lastRssiLoraSum = 0;
            printf("=====> The Last RSSI for End packet  = %d\n", lastRssiLora);

            startFlag = 0;
            endFlag = 1;

            memset(BleOtaHid,0,20);
            memcpy(BleOtaHid, hid, 12);
            BleOtaHid[12] = '\0';

puts("t7");
#ifdef SIMPLE_TIME_UPDATE
#ifdef CONFIG_HUB_WHITELIST
          } else if ((strcmp(cmd, cfgReqMessage) == 0) && !startFlag && (checkHubId(testId) != -1)) {
#else
          } else if ((strcmp(cmd, cfgReqMessage) == 0) && !startFlag) {
#endif //CONFIG_HUB_WHITELIST
#else
          } else if ((strcmp(cmd, cfgReqMessage) == 0) && !startFlag && (checkHubId(testId) != -1)) {
#endif //SIMPLE_TIME_UPDATE
            //if (hid_set == 0) {
            strcpy(hid, testId);
            //strncpy(hid + strlen(hid) - 1, "", 1);
            //hid_set = 1;
            //strncpy(curId, hid + 10, 2);
            //}
            startFlag = 0;

            printf("%d %d %s", counter, len, lora_buf[counter]+4);
            printf("HID = %s  curId = %s\n", hid, curId);

            respondUFlag = 1;
puts("t8");
          }
#ifdef TEMPERATURE_DECODING
            else if ((strcmp(cmd, temperatureMessage) == 0) && (strcmp(testId, curId) == 0)) {
            startFlag = 1;
            raw_time_get( &lastloratime );
            printf("%d %d", counter, len);//, lora_buf[counter]+4);
            printf("HID = %s  curId = %s\n", hid, curId);
            respondAFlag = 1;
            pch = strtok(NULL, ",");
            curSeqNum = atoi(pch);
            if (curSeqNum != lastSeqNum)
              counter++;
            lastSeqNum = curSeqNum;
          }
#endif //TEMPERATURE_DECODING
        }

      } // of rfm95.recv()
    }
      else
        printf("No data received\n");
puts("t9");
    } // of rfm95.available()

    if (respondAFlag || respondUFlag) {

      printf("Sending response to %s\n", hid);
      memset(bufff, 0, RH_RF95_MAX_MESSAGE_LEN);
      memcpy(bufff,networkHeader,4);
      if (respondUFlag) {
        //sprintf(bufff, "%s,U", hid);
        memset(hubCfgbuf, 0, HUB_CFG_MAX_LENGTH);
        createHubCfg(hid, hubCfgbuf);


        sprintf(bufff+4, "%s", hubCfgbuf);
        rf95.send((uint8_t *)bufff, 4+strlen(hubCfgbuf));
        //   printf("Sending response to %s\n", bufff);
        respondUFlag = 0;
puts("t10");
      } else {
        sprintf(bufff+4, "%s,A", hid);
        rf95.send((uint8_t *)bufff, 4+strlen(bufff+4));
        //    printf("Sending response to %s", bufff);
puts("t11");
      }
      // printf("Message : %s\n", bufff);
      //buff[0] = gps.location.lat(); buff[1] = gps.location.lng();
      //nrf_delay_ms(200);
      //usleep(1000*100);
      //ASrf95.send((uint8_t *)bufff, strlen(bufff));
      rf95.waitPacketSent();
      printf("lora Meesage Sent = %s\n", bufff+4);

#ifdef DEBUG
        write_Log("Conmet_GW", "Sending lora message", bufff+4, 0);
#endif
    }
  } // of While (!endFlag)

  printf("Number of frames received = %d\n", counter);
  if (counter >= 1) {
  printf("Sending Accel data to cloud\n");

     //if(counter != 1) loraBufDecode();

puts("t12");
     send_acceldata_to_cloud(hid, counter);
puts("t13");
  }

  printf("Lora Scan -- DONE\n");
}

void loraBufDecode() {
  //loraBufValues lora_buf
  int loraValCounter = 0;
  loraValTemperatureCounter = 0;
  int16_t tempVal;
  int i = 1;

  //TEMPERATURE MESSAGE DECODING
#ifdef TEMPERATURE_DECODING
  if(lora_buf[i][7] == 'T') {
    int8_t tempTemp = 0;
    for (int j = 11; (!((lora_buf[i][j] == '|') && (lora_buf[i][j+1] == 0)) && (j < 250)); j++) {
      loraBufTemperatureValues[loraValTemperatureCounter++] = (int8_t)lora_buf[i][j];
      //printf("temp buf %d = %02X\n", j, lora_buf[i][j], lora_buf[i][j]);
      //usleep(1000);
    }
    i++;

    for(int k = 0; k < loraValTemperatureCounter; k++) {
      printf("temperature value %d = %d\n", k, loraBufTemperatureValues[k]);
    }
  }
#endif //TEMPERATURE_DECODING


#ifdef HEXI
  for (; i < counter; i++) {
#ifdef TEMPERATURE_DECODING
    if (i < 11) {
#else //TEMPERATURE_DECODING
    if (i < 11) {
#endif //TEMPERATURE_DECODING

#ifdef INT16_BASED_READINGS
      printf("lora_count = %d\n", lora_buf_count[i]);
      int readingsPerMessage = (int)floorf((((float)lora_buf_count[i]-11.0)*8.0)/bitsPerMeasurement);
      printf("readings_per_message = %d\n", readingsPerMessage);
      int startBitOffset = 11*8;
      for(int j = 0; j < readingsPerMessage; j++) {
        //printf("here\n");
        tempVal = decode_bits_to_array(lora_buf[i], bitsPerMeasurement, (startBitOffset+(j*bitsPerMeasurement)));
        loraBufValues[loraValCounter++] = (float)((float)tempVal * scaleFactor);
        //printf("--V=%2.5f i=%d j=%d C=%d", loraBufValues[loraValCounter-1], i, j, loraValCounter-1);
        //usleep(10000);
      }

    } else {
      int readingsPerMessage = floorf((((float)lora_buf_count[i]-12.0)*8.0)/(float)bitsPerMeasurement);
      int startBitOffset = 12*8;
      for(int j = 0; j < readingsPerMessage; j++) {
        tempVal = decode_bits_to_array(lora_buf[i], bitsPerMeasurement, (startBitOffset+(j*bitsPerMeasurement)));
        loraBufValues[loraValCounter++] = (float)((float)tempVal * scaleFactor);
        //printf("--V=%2.5f i=%d j=%d C=%d", loraBufValues[loraValCounter-1], i, j, loraValCounter-1);
        //usleep(10000);
      }
    }
#else
      for (int j = 11; (!((lora_buf[i][j+2] == '|') && (lora_buf[i][j+3] == 0) && (lora_buf[i][j+4] == 0)) && (j < 248)); j = j + 2) {
        tempVal = (int16_t)(((int16_t)lora_buf[i][j + 1] << 8) | (int16_t)lora_buf[i][j]);
        loraBufValues[loraValCounter++] = (float)((float)tempVal / (float)500.0);
        //printf("--V=%2.3f i=%d j=%d C=%d", loraBufValues[loraValCounter-1], i, j, loraValCounter-1);
        //usleep(10000);
      }
    } else {
      for (int j = 12; (!((lora_buf[i][j+2] == '|') && (lora_buf[i][j+3] == 0) && (lora_buf[i][j+4] == 0)) && (j < 248)); j = j + 2) {
        tempVal = (int16_t)(((int16_t)lora_buf[i][j + 1] << 8) | (int16_t)lora_buf[i][j]);
        loraBufValues[loraValCounter++] = (float)((float)tempVal / (float)500.0);
        //printf("--V=%2.3f i=%d j=%d C=%d", loraBufValues[loraValCounter-1], i, j, loraValCounter-1);
        //usleep(10000);

      }
    }
#endif //INT16_BASED_READINGS
  }
#endif  //HEXI

#ifdef CHARI
  char *pch;
  for (; i < counter; i++) {
    memset(bufff,0,RH_RF95_MAX_MESSAGE_LEN);
    strcpy((char *)bufff, (char *)lora_buf[i]+4);
    pch = strtok((char *)bufff, ",");
    pch = strtok(NULL, ",");
    pch = strtok(NULL, ",");
    pch = strtok(NULL, ",");
    while(pch != NULL) {
      loraBufValues[loraValCounter++] = atof(pch);
      pch = strtok(NULL, ",|");
    }
  }

#endif

  printf("number of accel values decoded = %d\n", loraValCounter);

  if(loraValCounter > 0) {
    ofstream logfile;

    raw_time_get( &rawtime );
    timeinfo = localtime ( &rawtime );
    char logfileloc[100];
    memset(logfileloc,0,100);
    sprintf(logfileloc, "%sConmet_raw.txt", LOG_FILE_LOCATION, timeinfo->tm_mday, timeinfo->tm_mon, (timeinfo->tm_year+1900));
    logfile.open(logfileloc, ios::out);
    //logfile << "[" << setfill('0') << setw(2) << timeinfo->tm_hour
    //   << ":" << setfill('0') << setw(2) << timeinfo->tm_min
    //   << ":" << setfill('0') << setw(2) << timeinfo->tm_sec << "] "; //<< strNote << " - " << strToLog << endl;
#ifdef MULTI_AXIS
    for(int j = 0; j < loraValCounter; j++) {
      if((j%(loraValCounter/3) == 0) && (j != 0))
        logfile << "\n";
      logfile << loraBufValues[j] << ",";
    }
#else
    for(int j = 0; j < loraValCounter; j++)
      logfile << loraBufValues[j] << ",";
#endif //MULTI_AXIS
    logfile << endl;
    logfile.close();
  }

  //for(int k = 0; k <= loraValCounter; k++) {
  //  printf("accel value %d = %2.2f\n", k, loraBufValues[k]);
  //}
}

int checkHubId(char *hid) {
  for (int i = 0; i < Gateway.HubCnt; i++) {
    //printf("String 1 = %s\nString 2 = %s\n", Gateway.HubList[i].hubId, hid);
    if (strcmp(Gateway.HubList[i].hubId, hid) == 0) {
      //printf("hub index location = %d\n", i);
      return i;
    }
  }
  return -1;
}

time_t raw_time_get(time_t *sec) {
    return time(sec);
}

void write_Log(char *logName, char *strNote, const char *strToLog, int utc) {
  ofstream logfile;

  raw_time_get( &rawtime );
  if (utc)
      timeinfo = gmtime ( &rawtime );
  else
      timeinfo = localtime ( &rawtime );
  char logfileloc[200];
  sprintf(logfileloc, "%s%s_%d_%d_%d.txt", LOG_FILE_LOCATION, logName, timeinfo->tm_mday, timeinfo->tm_mon, (timeinfo->tm_year+1900));
  logfile.open(logfileloc, ios::app);
  logfile << "[" << setfill('0') << setw(2) << timeinfo->tm_hour
     << ":" << setfill('0') << setw(2) << timeinfo->tm_min
     << ":" << setfill('0') << setw(2) << timeinfo->tm_sec << "] " << strNote << " - " << strToLog << endl;
  logfile.close();
}

void setStartTime() {

  ifstream readfile(READ_FILE_LOCATION);

  if(readfile.is_open()) {
    string line;
    if(getline(readfile,line)) {
      int timea = atoi(line.c_str());
      time_o = (time_t)timea;
      printf("time = %ld\n", time_o);
    }
  }
}

int nbuf_delta()
{
    if (nbuf_pro >= nbuf_con)
        return nbuf_pro - nbuf_con;
    else
        return (NBUF_SIZE - nbuf_con) + nbuf_pro;
}

int nbuf_add(nbuf_call call, const char *adr, const char *buf, void (*cb)(char *res))
{
    nbuf[nbuf_pro].call = call;
    nbuf[nbuf_pro].adr = adr;
    void *p = malloc(strlen(buf) + 1);
    if (p == NULL) {
        printf("Error: nbuf_add: malloc: size: %d\n", strlen(buf) + 1);
        return 1;
    }
    strcpy((char *) p, buf);
    nbuf[nbuf_pro].buf = p;
    nbuf[nbuf_pro].cb = cb;

    if (++nbuf_pro >= NBUF_SIZE)
        nbuf_pro = 0;
    return 0;
}

void nbuf_process()
{
    int err = 0;
    int err2 = 0;
#ifndef TELIT
    char str[2048];
#endif //TELIT
    char respStr[512];
    memset(respStr,0,512);

    while (nbuf_delta() && !err) {
        if (nbuf[nbuf_con].call == NBUF_CALL_GET) {
#ifdef TELIT
            err = httpqry(&telit0, (const char *) nbuf[nbuf_con].buf, respStr, NULL);
#else
            strcpy(str, host_and_port);
            strcat(str, (const char *) nbuf[nbuf_con].buf);
            err = curlReq(str, respStr);
#ifdef MQTT
//todo: 2 x err
//todo: change gps from get to post
puts("GET ##########################################################################");
printf("nbuf[nbuf_con].buf: %s\r\n", nbuf[nbuf_con].buf);
            char *p = (char *) nbuf[nbuf_con].buf;
            while (*p != '?') p++;
            toJson(str, p);
            if (gps_data_set(&gps_data, str) == EXIT_SUCCESS) {
                puts("gps_data_set: OK");
                err2 = mqtt_send((void *) &gps_data, sizeof(ingest_gps_data_t));     //todo: implement mqtt_send status
#ifdef DEBUG
                write_Log("mqtt", "gps", err2 ? "Error" : "OK", 1);  //todo: add mqtt_send status
#endif //DEBUG
            } else
                puts("Error: gps_data_set");
#endif //MQTT
#endif //TELIT
            if (!err) {
                free(nbuf[nbuf_con].buf);
                if (nbuf[nbuf_con].cb)
                    (nbuf[nbuf_con].cb)(respStr);
                if (++nbuf_con >= NBUF_SIZE)
                    nbuf_con = 0;
            } else
                puts("Error: nbuf_process: get");
        } else if (nbuf[nbuf_con].call == NBUF_CALL_POST) {
#ifdef TELIT
            err = httpsnd(&telit0, nbuf[nbuf_con].adr,
              (char *) nbuf[nbuf_con].buf,
              strlen((const char *) nbuf[nbuf_con].buf), respStr, NULL);
#else
            strcpy(str, host_and_port);
            strcat(str, nbuf[nbuf_con].adr);
            err = curlPostReq(str, (char *)nbuf[nbuf_con].buf, respStr);
#ifdef MQTT
//todo: 2 x err
puts("POST ##########################################################################");
printf("nbuf[nbuf_con].adr: %s\r\n", nbuf[nbuf_con].adr);
            if (!strcmp(nbuf[nbuf_con].adr, "/addTempSet")) {
                if (temp_data_set(&temp_data, (const char *) nbuf[nbuf_con].buf) == EXIT_SUCCESS) {
                    puts("temp_data_set: OK");
                    err2 = mqtt_send((void *) &temp_data, sizeof(ingest_temp_data_t) - 2*(64 - temp_data.num_samples));  //todo: implement mqtt_send status
#ifdef DEBUG
                    write_Log("mqtt", "temp", err2 ? "Error" : "OK", 1);  //todo: add mqtt_send status
#endif //DEBUG
                } else
                    puts("Error: temp_data_set");
            }
            else if (!strcmp(nbuf[nbuf_con].adr, "/addMeasReadingCfgpost")) {
                if (sensor_status_data_set(&sensor_status_data, (const char *) nbuf[nbuf_con].buf) == EXIT_SUCCESS) {
                    puts("sensor_status_data_set: OK");
                    err2 = mqtt_send((void *) &sensor_status_data, sizeof(ingest_sensor_status_data_t));  //todo: implement mqtt_send status
#ifdef DEBUG
                    write_Log("mqtt", "sensor_status", err2 ? "Error" : "OK", 1);  //todo: add mqtt_send status
#endif //DEBUG
                } else
                    puts("Error: sensor_status_data_set");
            }
#endif //MQTT
#endif //TELIT
            if (!err) {
                free(nbuf[nbuf_con].buf);
                if (nbuf[nbuf_con].cb)
                    (nbuf[nbuf_con].cb)(respStr);
                if (++nbuf_con >= NBUF_SIZE)
                    nbuf_con = 0;
            } else
                puts("Error: nbuf_process: post");
#ifdef MQTT
        } else if (nbuf[nbuf_con].call == NBUF_CALL_MQTT) {
puts("MQTT ##########################################################################");
            if (!strcmp(nbuf[nbuf_con].adr, "/addMeasDataXYZ")) {
                if (vib_data_set(&vib_data, (const char *) nbuf[nbuf_con].buf) == EXIT_SUCCESS) {
                    puts("vib_data_set: OK");
                    err2 = mqtt_send((void *) &vib_data, sizeof(ingest_vib_data_t) - (sizeof(vib_data.value) - (3 * vib_data.num_samples * (((vib_data.flags >> 3) & 0x3) + 1))));  //todo: implement mqtt_send status
#ifdef DEBUG
                    write_Log("mqtt", "vib", err2 ? "Error" : "OK", 1);  //todo: add mqtt_send status
#endif //DEBUG
                } else
                    puts("Error: vib_data_set");
            }
            else
                puts("Error: nbuf_process: mqtt: unknown address");

            if (!err) {
                free(nbuf[nbuf_con].buf);
                if (nbuf[nbuf_con].cb)
                    (nbuf[nbuf_con].cb)(respStr);
                if (++nbuf_con >= NBUF_SIZE)
                    nbuf_con = 0;
            } else
                puts("Error: nbuf_process: mqtt");
#endif //MQTT
        } else
            puts("Error: nbuf_process: invalid call type");
    }
}

#ifdef TELIT
#define FTP_SERVER "45.79.227.160"
#define FTP_USER "gateway"
#define FTP_PASS "ywteg80/"
#define LOCAL_DIR "/home/gateway/ota"
#define REMOTE_DIR "/files/compass"

#define TRY2(f) {if(f) goto EXIT_WITH_ERROR;}

#define OTA_INTERVAL_DAYS 1

int ota(telit_t *telit, const char *ftp_server, const char *ftp_user,
    const char *ftp_pass, const char *local_dir, const char *remote_dir,
    int interval_days, int hour, const char *id)
{
    int err;
    int cnt = 0;

ACTIVATE:
    TRY2(telit->activate());

    err = telit->ota(ftp_server, ftp_user, ftp_pass, local_dir,
        remote_dir, interval_days, hour, id);
    cnt++;
    if (err) {
        if (err==2) {
            //reboot request
            return err;
        }
        if (cnt == 1) {
            puts("Error: ota. Retry with reset");
            telit->reset();
            goto ACTIVATE;
        } else if (cnt == 2) {
            puts("Error: ota. Retry with power cycle");
            TRY2(telit->power(TELIT_POWER_CYCLE));
            telit->reset();
            goto ACTIVATE;
        }
        puts("Error: ota. Give up");
        goto EXIT_WITH_ERROR;
    }
    return 0;

EXIT_WITH_ERROR:
    return 1;
}

int httpqry(telit_t *telit, const char *param, char *resp, size_t *resp_size)
{
    int err;
    int cnt = 0;

ACTIVATE:
    TRY(telit->activate());
    TRY(telit->cclkmode_set(CCLKMODE_UTC));

    err = telit->httpqry(param, resp, resp_size);
    cnt++;
    if (err) {
        if (cnt == 1) {
            puts("Error: httpqry. Retry with reset");
            telit->reset();
            goto ACTIVATE;
        } else if (cnt == 2) {
            puts("Error: httpqry. Retry with power cycle");
            TRY(telit->power(TELIT_POWER_CYCLE));
            telit->reset();
            goto ACTIVATE;
        }
        puts("Error: httpqry. Give up");
        goto EXIT_WITH_ERROR;
    }
    return 0;

EXIT_WITH_ERROR:
    return 1;
}

int httpsnd(telit_t *telit, const char *param, const char *data, size_t data_size,
        char *resp, size_t *resp_size)
{
    int err;
    int cnt = 0;

ACTIVATE:
    TRY(telit->activate());
    TRY(telit->cclkmode_set(CCLKMODE_UTC));

    err = telit->httpsnd(param, data, data_size, resp, resp_size);
    cnt++;
    if (err) {
        if (cnt == 1) {
            puts("Error: httpsnd. Retry with reset");
            telit->reset();
            goto ACTIVATE;
        } else if (cnt == 2) {
            puts("Error: httpsnd. Retry with power cycle");
            TRY(telit->power(TELIT_POWER_CYCLE));
            telit->reset();
            goto ACTIVATE;
        }
        puts("Error: httpsnd. Give up");
        goto EXIT_WITH_ERROR;
    }
    return 0;

EXIT_WITH_ERROR:
    return 1;
}
#endif //TELIT





int main(int argc, char *argv[])
{
   system("cd /dev:chmod 777 spidev0.0");
   //int fd;
    //disable stdout buffer
   setbuf(stdout, NULL);

   printf("%s started\n", argv[0]);
   char s[255];
   int i = 1;
   		//wiringPiSetup();
//all spi setups runs here
    //rf95.spi_setup();
   gps_uart_setup();
   //gps_spi_setup();

//    fd = wiringPiSPISetup(SPI_CHANNEL_LORA, 500000);
//    if (fd < 0) {
//        printf("Error: wiringPiSPISetup\n");
//        return 1;
//    }

    FILE * stream;
    stream = fopen("/home/torizon/json_test.txt", "r+");
    int count = fread(&buffer, sizeof(char), 500, stream);
    printf("count: %d\n", count);
    fclose(stream);

    //config_read();

    system("cp /usr/share/zoneinfo/UTC /etc/localtime");

#ifdef TELIT
    //config
    telit0.debug_print = 1;

    //power on
    TRY(telit0.power(TELIT_POWER_CYCLE));

    //reset
    telit0.reset();

    //init
    TRY(telit0.init(TELIT_SERIAL));

    //activate
    TRY(telit0.activate());

    //setup
    TRY(telit0.apn_set("wireless.twilio.com"));
    TRY(telit0.httpcfg_set("45.33.56.200", "80"));
    TRY(telit0.cclkmode_set(CCLKMODE_UTC));

    //set system date
    system("timedatectl set-ntp false");
    sleep(5);	//todo: make it synchronous
    TRY(telit0.date_get());
    char cmd[64];
    sprintf(cmd, "timedatectl set-time \"%d-%d-%d %d:%d:%d\"",
		    (telit0.date.tm_year+1900), telit0.date.tm_mon + 1, telit0.date.tm_mday,
		    telit0.date.tm_hour, telit0.date.tm_min, telit0.date.tm_sec);
    system(cmd);
    sleep(5);	//todo: make it synchronous

    //ota
    err = ota(&telit0, FTP_SERVER, FTP_USER, FTP_PASS, LOCAL_DIR,
      REMOTE_DIR, OTA_INTERVAL_DAYS, 7, config["GW_ID"].asCString());
    if (err==2) {
        //reboot request
        reboot();
    } else if (err)
        puts("Error: telit: ota");
    else
        puts("telit: ota: success");
#else
    err = system("timedatectl set-ntp true");
#endif //TELIT

#ifdef MQTT
    ssl_init();
#endif //MQTT

    printf("SIMPLE Gateway Started\n");
    write_Log("Conmet_GW", "SIMPLE Gateway started", defMessage, 0);
    sprintf(host_and_port, "http://23.239.22.240:8032");
    Gateway.HubCnt = -1;
    setStartTime();
    config_read("GW_ID", Gateway.macId);
    printf("MAC ID: %s", Gateway.macId);
    //strcpy(Gateway.macId, config["GW_ID"].asCString());
#ifdef CORPORATE_RETARGETING
    memset(host_and_port, 0, ENDPOINT_LENGTH);
    config_read("GW_CORPORATE_ADDRESS", host_and_port);
    //strcpy(host_and_port, config["GW_CORPORATE_ADDRESS"].asCString());
    strcat(host_and_port, ":");
    char port1[50];
    config_read("GW_CORPORATE_PORT", port1);
    //strcat(host_and_port, config["GW_CORPORATE_PORT"].asCString());
    strcat(host_and_port, port1);
    printf("corportate endpoint = %s\n", host_and_port);
#endif //CORPORATE_RETARGETING

#ifdef TELIT
#ifdef CORPORATE_RETARGETING
    char tempIP[20];
    char tempPort[6];
    memset(tempIP,0,20);
    memset(tempPort,0,6);
    sprintf(tempIP,"%s", strtok(host_and_port, ":"));
    sprintf(tempPort,"%s", strtok(NULL, ":"));
    printf("endpoint = %s:%s\n", tempIP, tempPort);
    if (telit0.httpcfg_set(tempIP, tempPort)) {
        printf("Error\n");
        return 1;
    }
    else printf("OK\n");
#else
    if (telit0.httpcfg_set("23.239.22.240", "8032")) {
        printf("Error\n");
        return 1;
    }
    else printf("OK\n");
#endif //CORPORATE_RETARGETING
#endif //TELIT

//get, retarget and store new endpoint
#ifdef CORPORATE_RETARGETING
    //Get the port to target from the corporate server
    char out_buff[128];
    memset(out_buff,0,128);
    char str[128];
    sprintf(str, "/getSysconfigFormac?mac=%s", Gateway.macId);
    printf("req str = %s\n", str);
#ifdef TELIT
    telit0.httpqry(str, out_buff, NULL);
#else
    strcpy(curl_arg_str, host_and_port);
    strcat(curl_arg_str, str);
    curlReq(curl_arg_str, out_buff);
#endif //TELIT
    printf("outbuf = %s, len = %d\n", out_buff, strlen(out_buff));

    //Parse and store the response from the server
    char *poch;
    //if(out_buff != NULL && (strcmp(out_buff, "\n") == 0)) {
//    if(strlen(out_buff) != 0) {
//      poch = strtok(out_buff, ",");
//      if(poch != NULL) {
//        config["GW_HTTP_ADDRESS"] = poch;
//        poch = strtok(NULL,",");
//        config["GW_HTTP_PORT"] = poch;
//        config_write();
//      }
//    }
//    config_read();

    //retarget systems to passed back target
    memset(host_and_port, 0, ENDPOINT_LENGTH);
    config_read("GW_HTTP_ADDRESS", host_and_port);
    //strcpy(host_and_port, config["GW_HTTP_ADDRESS"].asCString());
    strcat(host_and_port, ":");
    char port2[50];
    config_read("GW_HTTP_PORT", port2);
    //strcat(host_and_port, config["GW_HTTP_PORT"].asCString());
    strcat(host_and_port, port2);
    printf("customer endpoint = %s\n", host_and_port);

#ifdef TELIT
    memset(tempIP,0,17);
    memset(tempPort,0,6);
    sprintf(tempIP,"%s", strtok(host_and_port, ":"));
    sprintf(tempPort,"%s", strtok(NULL, ":"));
    printf("endpoint = %s:%s\n", tempIP, tempPort);
    if (telit0.httpcfg_set(tempIP, tempPort)) {
        printf("Error\n");
        return 1;
    }
    else printf("OK\n");
#endif //TELIT

#endif //CORPORATE_RETARGETING

    get_gateway_info();

#ifdef HUB_INFO_FROM_CLOUD

    char working_temp[50];
    memset(working_temp,0,50);

    if(Gateway.HubCnt == -1) {
      Gateway.HubCnt = atoi(config["HUB_COUNT"].asCString());
    } else {

      sprintf(working_temp, "%d", Gateway.HubCnt);

      config["HUB_COUNT"] = working_temp;

      for (int i = 0; i < Gateway.HubCnt; i++) {
        memset(working_temp,0,50);
        sprintf(working_temp, "HUB_%d", (i+1));
        config[working_temp] = Gateway.HubList[i].hubId;
      }


      config_write();
      config_read();
    }

#endif //HUB_INFO_FROM_CLOUD

#ifdef CONFIG_HUB_WHITELIST
    char tempstrs[15];
    memset(tempstrs, 0 , 15);

    Gateway.HubCnt = atoi(config["HUB_COUNT"].asCString());
    printf("hub count = %d\n", Gateway.HubCnt);

    for(int i = 0; i < Gateway.HubCnt; i++) {
      memset(tempstrs, 0 , 15);
      sprintf(tempstrs, "HUB_%d", (i+1));
      strcpy(Gateway.HubList[i].hubId, config[tempstrs].asCString());
      printf("hub %d id = %s\n", i+1, Gateway.HubList[i].hubId);
    }
#endif //CONFIG_HUB_WHITELIST

#ifdef QR_ID_UPLOADS

    //qr code from server
    memset(out_buff,0,128);
    memset(str,0,128);
    sprintf(str, "/mac-to-qr?gwaymacid=%s", Gateway.macId);
    printf("req str = %s\n", str);
#ifdef TELIT
    telit0.httpqry(str, out_buff, NULL);
#else
    strcpy(curl_arg_str, host_and_port);
    strcat(curl_arg_str, str);
    curlReq(curl_arg_str, out_buff);
#endif //TELIT
    printf("outbuf = %s, len = %d\n", out_buff, strlen(out_buff));

    if(strlen(out_buff) != 0) {
      poch = strtok(out_buff,  "\"");
      if(poch != NULL) {
        poch = strtok(NULL, "\"");
        if(poch != NULL) {
          if(strcmp(poch, "qrcode") == 0) {
            poch = strtok(NULL,  "\"");
            if(poch != NULL) {
              poch = strtok(NULL,  "\"");
              if(poch != NULL) {
                config["GW_QR"] = poch;
                config_write();
              }
            }
          }
        }
      }
    }
    config_read();

    printf("gateway QR = %s\n", config["GW_QR"].asCString());

#endif //QR_ID_UPLOADS

    raw_time_get( &prelogtime );


    printf("%s\n", argv[0]);
       int ret = 0;
       parse_opts(argc, argv);
       fd = open(device, O_RDWR);
    	if (fd < 0)
    	   pabort("can't open device");

    	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    	if (ret == -1)
    	pabort("can't set spi mode");

       ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
       if (ret == -1)
    	   pabort("can't get spi mode");

       /*
    	* bits per word
    	*/
       ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
       if (ret == -1)
    	   pabort("can't set bits per word");

       ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
       if (ret == -1)
    	   pabort("can't get bits per word");

       /*
    	* max speed hz
    	*/
       ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
       if (ret == -1)
    	   pabort("can't set max speed hz");

       ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
       if (ret == -1)
    	   pabort("can't get max speed hz");

       printf("spi mode: %d\n", mode);
       printf("bits per word: %d\n", bits);
       printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);


    loraInit();

    Gateway.SamplePeriod = 3600;

//get date and time
    raw_time_get( &rawtime );
//get gps
    err = gps_get(&Gateway.gps_data);
    if (err) {
        printf("Error: gps: %d\n", err);
        gps_invalid(&Gateway.gps_data);
//todo: this is only workaround for gps fix quality
        gps_fix = 0;
    }
    else {
        gps_fix = gps_fix_get(Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.satellites);
    }
//todo: this is only workaround for gps fix quality

    printf("Lat: %.7f Lon: %.7f\n", Gateway.gps_data.lat, Gateway.gps_data.lon);

    raw_time_get( &rawtime );

#ifdef QR_ID_UPLOADS
    sprintf(buf, "/heartbeatsync?name=%s&GQR=%s&ts=%d&gps=%.7f,%.7f,%.1f,%.1f,%.0f,%d,%.1f,%d&type=start",
		Gateway.macId, config["GW_QR"].asCString(), rawtime, Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.speed_mph, Gateway.gps_data.course_deg, Gateway.gps_data.altitude_feet, Gateway.gps_data.satellites, Gateway.gps_data.hdop/100.0, gps_fix);
#else
    sprintf(buf, "/heartbeatsync?name=%s&ts=%ld&gps=%.7f,%.7f,%.1f,%.1f,%.0f,%d,%.1f,%d&type=start",
		Gateway.macId, rawtime, Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.speed_mph, Gateway.gps_data.course_deg, Gateway.gps_data.altitude_feet, Gateway.gps_data.satellites, Gateway.gps_data.hdop/100.0, gps_fix);
#endif //QR_ID_UPLOADS

//add to nbuf
#ifdef SIMPLE_TIME_UPDATE
    err = nbuf_add(NBUF_CALL_GET, NULL, buf, parse_heartbeatInfo);
#else
    err = nbuf_add(NBUF_CALL_GET, NULL, buf, NULL);
#endif //SIMPLE_TIME_UPDATE
    if (err)
        printf("Error: nbuf_add: %d\n", err);
//process nbuf
    nbuf_process();

    while (1) {
            /*//if(BleOtaFlag) {
            write_Log("Conmet_GW", "Ble Ota triggered", "", 0);
            //MAC = BleOtaHid
            //CALL FUNCTION TO UPDATE AND UPLOAD FIRMWARE HERE
            sprintf(BleOtaHid, "D639E1535CE4
            printf("BleOtaHid: %s\n", BleOtaHid);
            char ble_addr_dfu[18];
            ble_addr_inc(ble_addr_dfu, BleOtaHid);
            char command[64];
            strcpy(command, "../ble/ble_dfu_test ");
            strcat(command, ble_addr_dfu);
            printf("dfu command: %s\n", command);
            system(command);
            BleOtaFlag = false;
        }*/

        lora_receiver();
        int state = readRegister(0x01);
        //printf("state = %d\n", state);

//	BleOtaFlag=false;
        /*if(BleOtaFlag) {
            write_Log("Conmet_GW", "Ble Ota triggered", "", 0);
            //MAC = BleOtaHid
            //CALL FUNCTION TO UPDATE AND UPLOAD FIRMWARE HERE
            printf("BleOtaHid: %s\n", BleOtaHid);
            char ble_addr_dfu[18];
            ble_addr_inc(ble_addr_dfu, BleOtaHid);
            char command[64];
            strcpy(command, "../ble/ble_dfu_test ");
            strcat(command, ble_addr_dfu);
            printf("dfu command: %s\n", command);
            system(command);
            BleOtaFlag = false;
        }*/

#ifdef SIMPLE_TIME_UPDATE
        if(heartbeatHit) {
            char buf[512];

//get date and time
            //raw_time_get();
//get gps
            err = gps_get(&Gateway.gps_data);
            if (err) {
                printf("Error: gps: %d\n", err);
                gps_invalid(&Gateway.gps_data);
//todo: this is only workaround for gps fix quality
                  gps_fix = 0;
              }
              else {
                  gps_fix = gps_fix_get(Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.satellites);
              }
//todo: this is only workaround for gps fix quality

            raw_time_get( &prelogtime );
            raw_time_get( &rawtime );

#ifdef QR_ID_UPLOADS
            sprintf(buf, "/heartbeatsync?name=%s&GQR=%s&ts=%d&gps=%.7f,%.7f,%.1f,%.1f,%.0f,%d,%.1f,%d&type=continue",
            Gateway.macId, config["GW_QR"].asCString(), rawtime, Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.speed_mph, Gateway.gps_data.course_deg, Gateway.gps_data.altitude_feet, Gateway.gps_data.satellites, Gateway.gps_data.hdop/100.0, gps_fix);
#else
            sprintf(buf, "/heartbeatsync?name=%s&ts=%ld&gps=%.7f,%.7f,%.1f,%.1f,%.0f,%d,%.1f,%d&type=continue",
            Gateway.macId, rawtime, Gateway.gps_data.lat, Gateway.gps_data.lon, Gateway.gps_data.speed_mph, Gateway.gps_data.course_deg, Gateway.gps_data.altitude_feet, Gateway.gps_data.satellites, Gateway.gps_data.hdop/100.0, gps_fix);
#endif //QR_ID_UPLOADS

            printf("Data to be send\n");
            printf("%s\n", buf);
            heartbeatHit = false;
//add to nbuf
#ifdef SIMPLE_TIME_UPDATE
            err = nbuf_add(NBUF_CALL_GET, NULL, buf, parse_heartbeatInfo);
#else
            err = nbuf_add(NBUF_CALL_GET, NULL, buf, NULL);
#endif //SIMPLE_TIME_UPDATE
            if (err)
                printf("Error: nbuf_add: %d\n", err);
//process nbuf
            nbuf_process();

            heartbeatHit = false;
        }
#endif //SIMPLE_TIME_UPDATE

//print nbuf delta
        printf("nbuf delta before: %d\n", nbuf_delta());

//process nbuf
        nbuf_process();

//print nbuf delta
        printf("nbuf delta after: %d\n", nbuf_delta());

//cron
        system("rm -f /run/watchdog");
        puts("1. watchdog cleared ---------------------------------------------------------------------------------------------");
        puts("2. watchdog cleared ---------------------------------------------------------------------------------------------");
        puts("3. watchdog cleared ---------------------------------------------------------------------------------------------");
        puts("4. watchdog cleared ---------------------------------------------------------------------------------------------");
        puts("5. watchdog cleared ---------------------------------------------------------------------------------------------");

#ifdef TELIT
        //ota
        err = ota(&telit0, FTP_SERVER, FTP_USER, FTP_PASS, LOCAL_DIR,
          REMOTE_DIR, OTA_INTERVAL_DAYS, 7, config["GW_ID"].asCString());
        if (err==2) {
            //reboot request
            reboot();
        } else if (err)
            puts("Error: telit: ota");
        else
            puts("telit: ota: success");
#endif //TELIT

    }

    close(fd);
    printf("done\n");
    return 0;
}
