#ifndef _NETDATA_H
#define _NETDATA_H

typedef struct 
{
    const char *word;
    const char *chinese;
}word_t;


word_t get_random_word();
bool get_net_word();
void net_data_task();
uint8_t get_weather();
#endif