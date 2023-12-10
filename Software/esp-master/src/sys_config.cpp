
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <sys_config.h>






sys_config_t sys_config;


/**
 * @brief 读取配置
 * 
 */
void read_config()
{
    // 打开文件
    DynamicJsonBuffer jsonBuffer;
    File file = LittleFS.open("/config.json", "r");
    JsonObject &config = jsonBuffer.parseObject(file.readString());
    sys_config.nightStart = config.get<bool>("nightStart");
    sys_config.radar = config.get<bool>("radar");
    sys_config.delayOff = config.get<unsigned char>("delayOff");
    sys_config.usbOut = config.get<bool>("usbOut");
    sys_config.word = config.get<bool>("word");
    
    JsonObject &port1 = config["port1"];
    sys_config.port1.sw = port1.get<bool>("switch");
    sys_config.port1.brightness = port1.get<unsigned char>("brightness");
    sys_config.port1.warm = port1.get<unsigned char>("warm");

    JsonObject &port2 = config["port2"];
    sys_config.port2.sw = port2.get<bool>("switch");
    sys_config.port2.brightness = port2.get<unsigned char>("brightness");
    sys_config.port2.warm = port2.get<unsigned char>("warm");

    JsonObject &port3 = config["port1"];
    sys_config.port3.sw = port3.get<bool>("switch");
    sys_config.port3.brightness = port3.get<unsigned char>("brightness");
    sys_config.port3.warm = port3.get<unsigned char>("warm");

    JsonObject &port4 = config["port1"];
    sys_config.port4.sw = port4.get<bool>("switch");
    sys_config.port4.brightness = port4.get<unsigned char>("brightness");
    sys_config.port4.warm = port4.get<unsigned char>("warm");
    Serial1.println();
    Serial1.printf("nightStart:%d, radar:%d, delayOff:%d, usbOut:%d, word:%d", sys_config.nightStart, sys_config.radar, sys_config.delayOff, sys_config.usbOut, sys_config.word);
    Serial1.println();
    Serial1.printf("port1.sw:%d, port1.brightness:%d, port1.warm:%d", sys_config.port1.sw, sys_config.port1.brightness, sys_config.port1.warm);
    Serial1.println();
    Serial1.printf("port2.sw:%d, port2.brightness:%d, port2.warm:%d", sys_config.port2.sw, sys_config.port2.brightness, sys_config.port2.warm);
    Serial1.println();
    Serial1.printf("port3.sw:%d, port3.brightness:%d, port3.warm:%d", sys_config.port3.sw, sys_config.port3.brightness, sys_config.port3.warm);
    Serial1.println();
    Serial1.printf("port4.sw:%d, port4.brightness:%d, port4.warm:%d", sys_config.port4.sw, sys_config.port4.brightness, sys_config.port4.warm);
    Serial1.println();
    // config.prettyPrintTo(Serial1);
    file.close();
}




sys_config_t get_sys_config()
{
    return sys_config;
}



/**
 * @brief 设置系统配置
 * 
 * @param sys_config 
 */
void set_sys_config(sys_config_t config)
{
    sys_config = config;
}



/**
 * @brief 系统配置修改
 * 
 */
void sys_congig_commit()
{
    DynamicJsonBuffer jsonBuffer;
    JsonObject& config = jsonBuffer.createObject();
    config["nightStart"] = !sys_config.nightStart ;
    config["radar"] = sys_config.radar;
    config["delayOff"] = sys_config.delayOff;
    config["usbOut"] = sys_config.usbOut;
    config["word"] = sys_config.word;

    JsonObject& port1   = jsonBuffer.createObject();
    port1["switch"]     = sys_config.port1.sw;
    port1["brightness"] = sys_config.port1.brightness;
    port1["warm"]       = sys_config.port1.warm;


    JsonObject& port2   = jsonBuffer.createObject();
    port2["switch"]     = sys_config.port2.sw;
    port2["brightness"] = sys_config.port2.brightness;
    port2["warm"]       = sys_config.port2.warm;


    JsonObject& port3   = jsonBuffer.createObject();
    port3["switch"]     = sys_config.port3.sw;
    port3["brightness"] = sys_config.port3.brightness;
    port3["warm"]       = sys_config.port3.warm;

    JsonObject& port4   = jsonBuffer.createObject();
    port4["switch"]     = sys_config.port4.sw;
    port4["brightness"] = sys_config.port4.brightness;
    port4["warm"]       = sys_config.port4.warm;

    config["port1"] = port1;
    config["port2"] = port2;
    config["port3"] = port3;
    config["port4"] = port4;

    File file = LittleFS.open("/config.json", "w");
    config.prettyPrintTo(file);
    file.close();
}
