/* this library is writing by  Omar Rosas.
 *     omar.rosas.calderon@gmail.com
 *  Designed to work with the LTE module Sim7600 series.
 *  
 *     This library use HardwareSerial, by default the pin is U2_RXD=16 U2_TXD=17.
 *     be sure that gnd is attached to ESP32 too. 
 *     
 *    
 *     
 *    
 *     
 *    PINOUT: 
 *        _____________________________
 *       |  ESP32 >>>   SIM7600G       |
 *        -----------------------------
 *            GND      >>>   GND
 *        U2_RXD  16   >>>   TX    
 *        U2_TXD  17   >>>   RX
 *       
 *                 
 *   POWER SOURCE 4.2V >>> VCC
 *
 *    Created on: September 21, 2021
 *        Author: Omar Rosas
 *        
 *
*/


#include <Arduino.h>

#ifndef Lte_SIM7600_h
#define Lte_SIM7600_h

#define SERIAL_SIZE_RX  1024
#define BAUD_RATE_H  115200

#ifndef LTE_YIELD_MS
#define LTE_YIELD_MS 0
#endif

#ifndef LTE_YIELD
#define LTE_YIELD() \
  { delay(LTE_YIELD_MS); }
#endif     


typedef const char* ConstStr;
 #define TIMEOUT_MS 60000L
// #define MQTT_TCP 0
// #define MQTT_SSL 1
// #define PORT_SSL 8883
// #define PORT_TCP 1883
// #define SSL_CTX_INDEX 1
// #define SESSION_ID 0
// #define DISABLE_CTX_SSL 0
// #define ENABLE_CTX_SSL 1
// #define SSL_VERSION 4
// #define SERVER_AUTH 1
// #define SERVER_CLIENT_AUTH 2
// #define CLIENT_AUTH 3
// #define ENABLE_INGORE_LOCAL_TIME 1
// #define DISENABLE_INGORE_LOCAL_TIME 1
// #define NEGOTIATE_TIME 300
enum RegSetValuesSSL{
	MQTT_TCP = 0,
	MQTT_SSL = 1,
	PORT_SSL = 8883,
	PORT_TCP = 1883,
	SSL_CTX_INDEX = 1,
	SESSION_ID = 0,
	DISABLE_CTX_SSL = 0,
	ENABLE_CTX_SSL = 1,
	SSL_VERSION = 4,
	SERVER_AUTH = 1,
	SERVER_CLIENT_AUTH = 2,
	CLIENT_AUTH = 3,
	ENABLE_INGORE_LOCAL_TIME = 1,
	DISENABLE_INGORE_LOCAL_TIME = 1,
	NEGOTIATE_TIME = 300,
};
enum RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class Sim7600G {
	private:
	public:
		Sim7600G();
		void begin(int baud_rate);
		bool restart();
		void powerDown();
		int readTemperatureModule();
		String readVoltagePowerSupplyModule();
		String getSignalQuality();
		String getModemInfo();
		String getRedInfo();
		int8_t isLteConnected();
		bool waitForNetwork(uint32_t timeout_ms = TIMEOUT_MS);		
};

class MqttClient {
	private:		
	public:
		void loop();
		bool connected();
    	bool connect(String cient_id, String user, String pass,String server, int32_t port, int8_t tcp_ssl = MQTT_TCP, int8_t contexSSL = DISABLE_CTX_SSL, int8_t sessionId = SESSION_ID, int8_t sslCtxIndex  = SSL_CTX_INDEX);
		int8_t subscribe(String topic, int16_t topic_length, int8_t qos);
		int8_t publish(String topic, String payload, int16_t topic_length, int16_t payload_length, int8_t qos);
		void clean();
		void setCallback(void(*puntero)(String topic, String payload));
};

class LteClientSecure {
	private:
	public:
		void setContext(String nameSet, int16_t modeValue, int8_t sslCtxIndex = SSL_CTX_INDEX );
		void setCert(String nameSet,  String nameCertFile, int8_t sslCtxIndex = SSL_CTX_INDEX);
		void certDown(String nameCert , String cert, int32_t lenCert);
		int8_t contect( String server, int32_t port, int8_t sessionId = SESSION_ID , int8_t sslCtxIndex = SSL_CTX_INDEX );
		void close(int8_t sessionId = SESSION_ID);
		void deleteCert(String nameCert);
		bool listCert();
		void setCACert(String root_ca, int8_t sslVersion = SSL_VERSION, int8_t authMode = SERVER_AUTH , int8_t ignoreLocalTime = ENABLE_INGORE_LOCAL_TIME, int16_t negotiateTime = NEGOTIATE_TIME);	
};

class HttpsClient {
	private:
	public:
		void begin(String url, int8_t contexSSL = DISABLE_CTX_SSL,  int8_t sslCtxIndex  = SSL_CTX_INDEX);
		void addHeader(String type_header, String header);
		int16_t  post(String data);
		String getString();
		void end();
};

#endif
