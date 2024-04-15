#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ntp.h>
#include <ESPWiFiManager.h>
#include <wifi.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com", 8*3600, 60000);


void ntp_init()
{
    timeClient.begin();
}


void ntp_update()
{
    timeClient.forceUpdate();
}


 time_t getNtpTime()
{

    time_t timep = timeClient.getEpochTime();
    struct tm *p;
    p = gmtime(&timep);
    // Serial.printf("year:%d, %d", p->tm_year, wifi_isconnect() ? 1 : 0);
    if (p->tm_year   == 70 && wifi_isconnect())
    {
        timeClient.forceUpdate();
    }
    // return String((1900 + p->tm_year)) + "-" + String((1 + p->tm_mon)) + "-" + String(p->tm_mday) + " " + String(timeClient.getFormattedTime());
    return timep;
}