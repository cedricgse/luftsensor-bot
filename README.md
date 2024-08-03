Air Quality Sensor with Telegram Bot using ESP32

## config.h
```
#ifndef CONFIG
#define CONFIG

const char* ssid = "";
const char* wlan_pwd = "";
const char* BotToken = "";
const char* chat_ID = "";

#endif
```

The BSEC library require modifcation to the espressif8266 framework in order to avoid linking errors. Check [BSEC-Arduino-library|https://github.com/BoschSensortec/BSEC-Arduino-library] for details. But shortly - locate the eagle.app.v6.common.ld file and put *libalgobsec.a:(.literal.* .text.*) line below the *libwps.a:(.literal.* .text.*) line.