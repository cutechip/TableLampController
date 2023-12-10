#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ntp.h>


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8*3600, 60000);


void ntp_init()
{
    timeClient.begin();
}


void ntp_update()
{
    timeClient.update();
}


 time_t getNtpTime()
{

    time_t timep = timeClient.getEpochTime();
    // struct tm *p;
    // p = gmtime(&timep);
    // return String((1900 + p->tm_year)) + "-" + String((1 + p->tm_mon)) + "-" + String(p->tm_mday) + " " + String(timeClient.getFormattedTime());
    return timep;
}