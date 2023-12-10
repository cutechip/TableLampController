#ifndef _WIFI_H
#define _WIFI_H

    void wifi_init();
    void auto_connect_wifi();
    void wifi_process_task();
    boolean wifi_isconnect();
    wifi_connect_status_t get_wifi_connect_status();
    
#endif