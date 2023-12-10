#ifndef _IOTCLIENT_H
#define _IOTCLIENT_H

void mqtt_callback(char* topic, byte* payload, unsigned int length);
String getIotPassword(String username);

#endif