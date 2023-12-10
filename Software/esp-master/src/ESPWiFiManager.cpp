#include "ESPWiFiManager.h"
#include "LittleFS.h"


ESPWiFiManager::ESPWiFiManager() {


}
   

ESPWiFiManager::~ESPWiFiManager()
{
   
}

template <typename Generic>
void ESPWiFiManager::DEBUG_WM(Generic text) {
  if (_debug) {
    Serial1.print("*WM: ");
    Serial1.println(text);
  }
}

void ESPWiFiManager::getWifiList()
{
    String page = "[";
    int n = WiFi.scanNetworks();
    DEBUG_WM(F("Scan done"));
    if (n == 0) {
      DEBUG_WM(F("No networks found"));
    } else {
      //sort networks
      int indices[n];
      for (int i = 0; i < n; i++) {
        indices[i] = i;
      }

      // RSSI SORT

      // old sort
      for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
          if (WiFi.RSSI(indices[j]) < WiFi.RSSI(indices[i])) {
            std::swap(indices[i], indices[j]);
          }
        }
      }

        String cssid;
        for (int i = 0; i < n; i++) {
          if (indices[i] == -1) continue;
          cssid = WiFi.SSID(indices[i]);
          for (int j = i + 1; j < n; j++) {
            if (cssid == WiFi.SSID(indices[j])) {
              DEBUG_WM("DUP AP: " + WiFi.SSID(indices[j]));
              indices[j] = -1; // set dup aps to index -1
            }
          }
        }
        
      //display networks in page
      for (int i = 0; i < n; i++) {
        if (indices[i] == -1) continue; // skip dups
        DEBUG_WM(WiFi.SSID(indices[i]));
        DEBUG_WM(WiFi.RSSI(indices[i]));
        int quality = getRSSIasQuality(WiFi.RSSI(indices[i]));
        if (i != 0) page += ",";
        String rssiQ;
        rssiQ += quality;
        page += "{\"name\":\"" + WiFi.SSID(indices[i]) + "\",\"rssi\":" + rssiQ +"}";
       
      }
      page += "]";
    }
    server->sendHeader("Content-Length", String(page.length()));
    server->send(200, "text/html", page);
}



void ESPWiFiManager::autoConnect() {
  String ssid = "ESP" + String(ESP.getChipId());
  return autoConnect(ssid.c_str(), NULL);
}

void ESPWiFiManager::autoConnect(char const *apName, char const *apPassword) {
  DEBUG_WM(F(""));
  DEBUG_WM(F("AutoConnect"));

  // attempt to connect; should it fail, fall back to AP
  WiFi.mode(WIFI_STA);
  startConfigPortal(apName, apPassword);
}


/**
 * @brief 保存WIFI信息
 * 
 */
void ESPWiFiManager::saveWifiInfo() {
    DEBUG_WM(F("WiFi save"));

    //SAVE/connect here
    _ssid = server->arg("ssid").c_str();
    _pass = server->arg("pwd").c_str();

    DEBUG_WM(_ssid);
    DEBUG_WM(_pass);

    String page = "正在连接,请稍后。。。。";

    server->sendHeader("Content-Length", String(page.length()));
    server->send(200, "text/html", page);
    wifi_setp = ESPWIFI_CHECK_CONNECT;
    if (_connectCallback != NULL) 
    {
        _connectCallback(WIFICONFIG_RECV_WIFI_INFO);
    } 
}

void ESPWiFiManager::handleNotFound() {
  if (captivePortal()) { // If captive portal redirect instead of displaying the error page.
    return;
  }
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server->uri();
  message += "\nMethod: ";
  message += ( server->method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server->args();
  message += "\n";

  for ( uint8_t i = 0; i < server->args(); i++ ) {
    message += " " + server->argName ( i ) + ": " + server->arg ( i ) + "\n";
  }
  server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->sendHeader("Content-Length", String(message.length()));
  server->send ( 404, "text/plain", message );
}


/** Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the request again. */
boolean ESPWiFiManager::captivePortal() {
  if (!isIp(server->hostHeader()) ) {
    DEBUG_WM(F("Request redirected to captive portal"));
    server->sendHeader("Location", String("http://") + toStringIp(server->client().localIP()), true);
    server->send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    server->client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

void ESPWiFiManager::setupConfigPortal() {
	dnsServer.reset(new DNSServer());
	server.reset(new ESP8266WebServer(80));

	DEBUG_WM(F(""));
	DEBUG_WM(F("Configuring access point... "));
	DEBUG_WM(_apName);
	if (_apPassword != NULL) {
		if (strlen(_apPassword) < 8 || strlen(_apPassword) > 63) {
		// fail passphrase to short or long!
		DEBUG_WM(F("Invalid AccessPoint password. Ignoring"));
		_apPassword = NULL;
		}
		DEBUG_WM(_apPassword);
	}

	if (_apPassword != NULL) {
		WiFi.softAP(_apName, _apPassword);
	} else {
		WiFi.softAP(_apName);
	}

	delay(500); // Without delay I've seen the IP address blank
	DEBUG_WM(F("AP IP address: "));
	DEBUG_WM(WiFi.softAPIP());
	dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
	dnsServer->start(DNS_PORT, "*", WiFi.softAPIP());

    server->serveStatic("/", LittleFS, "index.html");
    server->on(String(F("/getWifiList")).c_str(), std::bind(&ESPWiFiManager::getWifiList, this));
    server->on(String(F("/save")).c_str(), std::bind(&ESPWiFiManager::saveWifiInfo, this));
    server->onNotFound (std::bind(&ESPWiFiManager::handleNotFound, this)); 
    server->begin(); // Web server start
    DEBUG_WM(F("HTTP server started"));
    if (_connectCallback != NULL)
    {
        _connectCallback(WIFICONFIG_INIT_SUCCESS);
    }
}



void  ESPWiFiManager::startConfigPortal(char const *apName, char const *apPassword) {
  
  if(!WiFi.isConnected()){
    WiFi.persistent(false);
    WiFi.disconnect(); 
    WiFi.mode(WIFI_AP);
    WiFi.persistent(true);
  }
  else {
    //setup AP
    WiFi.mode(WIFI_AP_STA);
    DEBUG_WM(F("SET AP STA"));
  }
  _apName = apName;
  _apPassword = apPassword;
  wifi_setp = ESPWIFI_WAIT_RECVINFO;
  startConnectTick = millis();
  setupConfigPortal();
}







/**
 * @brief 设置连接回调函数
 * 
 */
void ESPWiFiManager::setConnectCallback(void (*connectCallback)(wifi_connect_status_t))
{
    _connectCallback = connectCallback;
}


void ESPWiFiManager::checkWifiConnect()
{
	// 判断当前连接的wifi是否是我们将要连的WIFI,如果是 我们就直接退出
	if (WiFi.status() == WL_CONNECTED && (WiFi.SSID() == _ssid)) 
    {
        return ;
    }
	if (_ssid != "") 
    {
        ETS_UART_INTR_DISABLE();
        wifi_station_disconnect();
        ETS_UART_INTR_ENABLE();
        WiFi.begin(_ssid.c_str(), _pass.c_str(), 0, NULL, true);
    }
}


/**
 * @brief 查询上一次有误保存的wifi
 * 
 * @return int 
 */
uint8_t  ESPWiFiManager::connectLastWifi()
{
    if (WiFi.SSID() != "") 
    {
		
        DEBUG_WM(F("Using last saved values, should be faster"));
		DEBUG_WM(WiFi.SSID().c_str());
        ETS_UART_INTR_DISABLE();
        wifi_station_disconnect();
        ETS_UART_INTR_ENABLE();
        WiFi.begin();
        return 1;
    } 
    else 
    {
        return 0;
    }
}

/**
 * @brief WIFIl连接处理
 * 
 */
void ESPWiFiManager::wifiHandle() 
{
	static uint32_t wifiTick = millis();

	switch (wifi_setp)
	{
	case ESPWIFI_IDLE:
		break;
    case ESPWIFI_INIT:
        if (connectLastWifi())
        {
            wifi_setp = ESPWIFI_WAIT_CONNECT;
            startConnectTick = millis();
            _connectTimeout = 10 * 1000;
            return ;
        }
        wifi_setp = ESPWIFI_IDLE;
        break;
	case ESPWIFI_WAIT_RECVINFO:
		dnsServer->processNextRequest();
		server->handleClient();
		break;
	case ESPWIFI_CHECK_CONNECT:
		checkWifiConnect();
		wifi_setp = ESPWIFI_WAIT_CONNECT;
		break;
	case ESPWIFI_WAIT_CONNECT:
		uint8_t status;
		if (millis() - wifiTick > 1000)
		{
			wifiTick = millis();
			status = WiFi.status();
			// Serial1.printf("status:%d", status);
			if (status == WL_CONNECTED && !wifi_connect_status)
			{
                wifi_connect_status = 1;
                _connectTimeout = 0;
                // 连接成功后设置为STA模式
                WiFi.mode(WIFI_STA);
                server.reset();
                dnsServer.reset();
				        DEBUG_WM(F("wifi connect success....."));
                if (_connectCallback != NULL) 
                {
                    _connectCallback(WIFICONFIG_CONNECT_SUCCESS);
                }   
			} 
            else if (status == WL_DISCONNECTED && wifi_connect_status)
            {
                wifi_connect_status = 0;
                if (_connectCallback != NULL) 
                {
                    _connectCallback(WIFICONFIG_DISCONNECTED);
                }  
            }
		}

        // 超时检测
        if (_connectTimeout > 0 && (millis() - startConnectTick > _connectTimeout))
        {
            _connectTimeout = 0;
            // 连接超时 设置为STA模式
            WiFi.mode(WIFI_STA);
            server.reset();
            dnsServer.reset();
            if (_connectCallback != NULL) 
            {
                _connectCallback(WIFICONFIG_CONNECT_TIMEOUT);
            }  
        }
		break;
	default:
		break;
	}
}


void ESPWiFiManager::setHostname(String hostname)
{
    WiFi.hostname(hostname);
}
 /**
  * @brief 设置连接超时
  * 
  * @param seconds 
  */
void ESPWiFiManager::setTimeout(unsigned long seconds)
{
    _connectTimeout = seconds * 1000;
}

int ESPWiFiManager::getRSSIasQuality(int RSSI) {
  int quality = 0;

  if (RSSI <= -100) {
    quality = 0;
  } else if (RSSI >= -50) {
    quality = 100;
  } else {
    quality = 2 * (RSSI + 100);
  }
  return quality;
}

/** IP to String? */
String ESPWiFiManager::toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}


/** Is this an IP? */
boolean ESPWiFiManager::isIp(String str) {
  for (size_t i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}