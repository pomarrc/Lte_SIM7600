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
#include <HardwareSerial.h>
#include <Lte_SIM7600.h>
HardwareSerial LTE(1);




//function declaration
void (*punteroCallback)(String topic, String payload);
template <typename... Args>
void sendAT(Args... cmd);
template <typename T>
void streamWrite(T last);
template <typename T, typename... Args>
void streamWrite(T head, Args... tail);
int8_t waitResponse(uint32_t timeout_ms, String& data, const char* c1 =  LTE_OK, const char* c2 = LTE_ERROR);
int8_t waitResponse(uint32_t timeout_ms, const char* c1 = LTE_OK, const char* c2 = LTE_ERROR);
int8_t waitResponse(const char* c1 = LTE_OK, const char* c2 = LTE_ERROR);
inline int16_t streamGetIntBefore(char lastChar);
inline bool streamSkipUntil(const char c, const uint32_t timeout_ms = 1000L);




Sim7600G::Sim7600G()
{

}
//STATUS CONTROL
void Sim7600G::begin(int baud_rate){
  LTE.begin(baud_rate,SERIAL_8N1,16,17, false);
  LTE.setRxBufferSize(SERIAL_SIZE_RX); 
}

bool Sim7600G::restart(){
  for (uint32_t start = millis(); millis() - start < 1000L;) {
      sendAT("");
      if (waitResponse(200) == 1) { 
        sendAT("+CRESET");
        if (waitResponse(10000L) != 1) { return false; }
        delay(5000L);
        return true;
      }
      delay(100);
    }
    return false;
}

void Sim7600G::powerDown(){
  sendAT("+CPOF");
  waitResponse();
}

String Sim7600G::getSignalQuality(){
  sendAT("+CSQ");
  if(waitResponse(1000L, "+CSQ:"   ) != 1){ }
  String quality = LTE.readStringUntil('\n');
  quality.trim();
  waitResponse();
  return quality;
}

String Sim7600G::getModemInfo(){
  sendAT("+CGMM");
  String model = LTE.readStringUntil('\n');
  model.trim();
  waitResponse();
  return model;
}
//HARDWARE RELATED
int Sim7600G::readTemperatureModule(){////////verificar es float
  sendAT("+CPMUTEMP");
  if(waitResponse(1000L, "+CPMUTEMP:") != 1){ }
  String temperature = LTE.readStringUntil('\n');
  temperature.trim();
  int temp = temperature.toInt();
  waitResponse();
  return temp;
}

String Sim7600G::readVoltagePowerSupplyModule(){
   sendAT("AT+CBC");
  if(waitResponse(1000L, "+CBC") != 1){ }
  String voltage = LTE.readStringUntil('\n');
  voltage.trim();
  waitResponse();
  return voltage;
}
//NETWORK LTE
int8_t Sim7600G::isLteConnected(){  
  return _getRegistrationStatus();
}

bool Sim7600G::waitForNetwork(uint32_t timeout_ms) {
    return _waitForNetwork(timeout_ms);
  }

String Sim7600G::getRedInfo(){
  sendAT("+COPS?");
  if(waitResponse(1000L, "+COPS:") != 1){ }
  String redInfo = LTE.readStringUntil('\n');
  redInfo.trim();
  waitResponse();
  return redInfo;
}
 /*
* Private network methods
*/
bool Sim7600G::_waitForNetwork(uint32_t timeout_ms ) {
   
    for (uint32_t start = millis(); millis() - start < timeout_ms;) {
      if (_isNetworkConnected()) { return true; }
      delay(250);
    }
    return false;
  }
bool Sim7600G::_isNetworkConnected(){
  int s = _getRegistrationStatus();
    return (s == REG_OK_HOME || s ==  REG_OK_ROAMING);
  }
int8_t Sim7600G::_getRegistrationStatus(){
    sendAT("+CREG?");
    int8_t resp= waitResponse(1000L, "+CREG:");
    if (resp != 1) { return -1; }
    streamSkipUntil(','); /* Skip format (0) */
    int status = streamGetIntBefore('\n');
    //Serial.print("status reg  -->");Serial.println(status);
    waitResponse();
    return status;
  }
/*
* End private network methods
*/
//MQTT(S)
bool MqttClient::connected(){
  int response ;
  int length;
  sendAT("+CMQTTDISC?");
  if(waitResponse(6000L, "+CMQTTDISC:") != 1){   }
  String res = LTE.readStringUntil('\n');
  length = res.indexOf(",");
  res = res.substring(length+1);
  length = res.indexOf('\n');
  res.trim();  
  response = res.toInt();
  if(response != 0){
     return false;
   }
   if(waitResponse(1000L, "+CMQTTDISC: 1,1") != 1){   }//catch +CMQTTDISC: 1,1
   waitResponse();//catch OK
  return true;
}

bool MqttClient::connect(String cient_id,  String user, String pass, String server, int32_t port, int8_t tcp_ssl, int8_t contexSSL,int8_t sessionId , int8_t sslCtxIndex ){
  int response;
  int length;
  String broker = "tcp://" +  server + ":" + String(port) ;
  sendAT("E0");
  waitResponse();
  sendAT("+CMQTTSTART");
  waitResponse();
  sendAT("+CMQTTACCQ=0,\"", cient_id, "\"", "," ,String(tcp_ssl));
  waitResponse();
  if(contexSSL == 1){
     sendAT("AT+CMQTTSSLCFG=", sessionId, ",", sslCtxIndex); 
     waitResponse();
  }   
  sendAT("+CMQTTCONNECT=0,\"", broker, "\"", ",90,1,","\"", user, "\",\"",  pass,"\"");
  if(waitResponse(6000L, "+CMQTTCONNECT:") != 1){  response = 1; }
  String res = LTE.readStringUntil('\n');
  length = res.indexOf(",");
  res = res.substring(length+1);
  length = res.indexOf('\n');
  res.trim();  
  response = res.toInt();
  if(response != 0){
     return false;
   }
  return true;
}

int8_t MqttClient::subscribe(String topic, int16_t topic_length, int8_t qos){
  
  sendAT("+CMQTTSUBTOPIC=0,",String(topic_length),",",String(qos));
  if ( waitResponse(1000L, ">") == 1) { 
    LTE.print(topic);  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    waitResponse();
  }
  sendAT("+CMQTTSUB=0,5,0,0");//AT+CMQTTSUB=<client_index>,<reqLength>,<qos>[,<dup>]
  if ( waitResponse(1000L, ">") == 1) { 
    LTE.print("test");  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    waitResponse();
  }
  if(waitResponse(1000L, "+CMQTTSUB:") != 1){   }//catch Error 
  streamSkipUntil(','); /* Skip format (0) */
  int8_t error = streamGetIntBefore('\n');
  return error;
}

int8_t MqttClient::publish(String topic, String payload, int16_t topic_length, int16_t payload_length, int8_t qos){
  
  sendAT("+CMQTTTOPIC=0,", String(topic_length));
  if ( waitResponse(1000L, ">") == 1) { 
    LTE.print(topic);  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    waitResponse(); 
  }
  sendAT("+CMQTTPAYLOAD=0,",  String(payload_length));
  if ( waitResponse(1000L, ">") == 1) { 
    LTE.print(payload);  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    waitResponse();
  }
  sendAT("+CMQTTPUB=0,",  String(qos), ",60");
   if(waitResponse(1000L, "+CMQTTPUB:") != 1){   }//catch Error 
   streamSkipUntil(','); /* Skip format (0) */
   int8_t error = streamGetIntBefore('\n');
   return error;
    
}

void MqttClient::clean(){
  sendAT("+CMQTTDISC=0,60");
  waitResponse();
  sendAT("+CMQTTREL=0");
  waitResponse();
  sendAT("+CMQTTSTOP");
  waitResponse();
}

void MqttClient::loop(){
  String topic, payload;
  String res;
  if(waitResponse(1000L, "+CMQTTRXSTART:") == 1){ 
    res = LTE.readStringUntil('\n');
    if(waitResponse(1000L, "+CMQTTRXTOPIC:") == 1){  
      topic = LTE.readStringUntil('+');
      int length = topic.indexOf('\n');
      topic = topic.substring(length);
      topic.trim();
      if(waitResponse(1000L, "CMQTTRXPAYLOAD:") == 1){ 
        payload = LTE.readStringUntil('+');
        int length1 = payload.indexOf('\n');
        payload = payload.substring(length1);
        payload.trim();
        if(payload.length()> 0 ){
          punteroCallback( topic,  payload);
        }  
      }
    }
   }  
}

void MqttClient::setCallback(void(*puntero)(String topic, String payload))
{
   punteroCallback = puntero;
}

//HTTP(S)
void HttpsClient::begin(String url ,int8_t contexSSL,  int8_t sslCtxIndex ){
  sendAT("+HTTPINIT");
  waitResponse();
  sendAT("+HTTPPARA=\"URL\",\"", url, "\""); 
  waitResponse();
  if(contexSSL == 1){
     sendAT("+HTTPPARA=\"SSLCFG\",", sslCtxIndex); 
     waitResponse();
  }      
}

void HttpsClient::addHeader(String type_header, String header){
  if (type_header =="Content-Type"){
     sendAT("+HTTPPARA=\"CONTENT\",\"", header, "\""); 
     waitResponse();  
  }
}

int16_t HttpsClient::post(String data){
  int16_t size_data = data.length();
  sendAT("+HTTPDATA=", String(size_data), ",1000"); 
  if ( waitResponse(1000L, "DOWNLOAD") == 1) { 
    LTE.print(data);  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    waitResponse();
  }
  sendAT("+HTTPACTION=1");
  waitResponse();//recibe OK
  if(waitResponse(6000L, "+HTTPACTION:")!=1){ return 0; }
  String res = LTE.readStringUntil('\n');
  int16_t length = res.indexOf(',');
  res = res.substring(length+1,length+4);
  return res.toInt();
}

String HttpsClient::getString(){
  String response;
  int length;
  sendAT("+HTTPHEAD");
  if(waitResponse(1000L, "Content-Length:") == 1){ 
  response = LTE.readStringUntil('\n');
  response.trim();  
  }
  String Content_Length = response;
  sendAT("+HTTPREAD=0,", Content_Length);
  if(waitResponse(1000L, "+HTTPREAD:") != 1){ response =""; }
  if(waitResponse(1000L, "DATA,") != 1){ response =""; }
  response = LTE.readStringUntil('+');
  length = response.indexOf('\n');
  response = response.substring(length);
  response.trim();
  return response;       
}

void HttpsClient::end(){
   sendAT("+HTTPTERM");
   waitResponse();
}

//SSL/TLS   
void LteClientSecure::setCACert(String root_ca, int8_t sslVersion, int8_t authMode, int8_t ignoreLocalTime, int16_t negotiateTime){
  _configureSSlContext("sslversion", sslVersion);
  _configureSSlContext("authmode", authMode);
  _configureSSlContext("ignorelocaltime", ignoreLocalTime);
  _configureSSlContext("negotiatetime", negotiateTime);
  if(_listCert()){
     _deleteCert("ca.pem");
  }
  int16_t len = root_ca.length();
  _certDown("ca.pem", root_ca, len);
  _setCert("cacert","ca.pem");
}

void LteClientSecure::setContext(String nameSet,  int16_t modeValue, int8_t sslCtxIndex ){
  _configureSSlContext( nameSet, modeValue, sslCtxIndex );
}
         
void LteClientSecure::setCert(String nameSet,  String nameCertFile, int8_t sslCtxIndex ){
  _setCert(nameSet, nameCertFile, sslCtxIndex );
}

void LteClientSecure::certDown(String nameCert , String cert , int32_t lenCert ){   
  _certDown(nameCert, cert, lenCert );
}

int8_t LteClientSecure::contect(String server, int32_t port, int8_t sessionId ,int8_t sslCtxIndex){
  return _contectSSL( server, port, sessionId , sslCtxIndex); 
}

void LteClientSecure::close(int8_t sessionId){
  _closeSSL(sessionId); 
}

void LteClientSecure:: deleteCert(String nameCert){
  _deleteCert(nameCert);
}

bool LteClientSecure:: listCert(){
   return _listCert();
}
 /*
* Private LteClientSecure methods
*/
void LteClientSecure:: _configureSSlContext(String nameSet,  int16_t modeValue, int8_t sslCtxIndex ){
  sendAT("+CSSLCFG=\"", nameSet,"\"", ",", String(sslCtxIndex),",", String(modeValue));
  waitResponse();
}
void LteClientSecure::  _setCert(String nameSet,  String nameCertFile, int8_t sslCtxIndex){
  sendAT("+CSSLCFG=\"", nameSet,"\"", ",", String(sslCtxIndex), ",", "\"", nameCertFile, "\"");
  waitResponse();
}
void LteClientSecure:: _certDown(String nameCert , String cert , int16_t lenCert ){   
  sendAT("+CCERTDOWN=\"" ,nameCert,"\"" , ",", String(lenCert));
  if ( waitResponse(">") == 1) { 
    LTE.print(cert);  // Actually send the message
    LTE.write(static_cast<char>(0x1A));  // Terminate the message
    LTE.flush();
    if(waitResponse(6000L) != 1){ 
       Serial.println("down cert ERROR" );
      }  
  }
}
int8_t LteClientSecure:: _contectSSL(String server, int32_t port, int8_t sessionId ,int8_t sslCtxIndex){
  sendAT("+CCHSTART");
  waitResponse();
  sendAT("+CCHSSLCFG=", String(sessionId),",", String(sslCtxIndex));
  waitResponse();
  sendAT("+CCHOPEN=", String(sessionId), ",", "\"", server, "\"",",", String(port));
  if(waitResponse(1000L, "+CCHOPEN:") != 1){ }
  streamSkipUntil(','); /* Skip format (0) */
  int8_t status = streamGetIntBefore('\n');
   return status;
}
void LteClientSecure:: _closeSSL(int8_t sessionId){
  sendAT("+CCHCLOSE=", String(sessionId));
  waitResponse();
  sendAT("+CCHSTOP");
  waitResponse();
}
bool LteClientSecure:: _listCert(){
  sendAT("+CCERTLIST");
  if(waitResponse(1000L, "+CCERTLIST:") != 1){return false; }
   String response = LTE.readStringUntil('\n');
   return true;
}
void LteClientSecure:: _deleteCert(String nameCert){
  sendAT("+CCERTDELE=\"", nameCert, "\"");
  waitResponse();
} 
/*
* End private LteClientSecure methods
*/
//FTP(S)

//GNSS

//SMS

//CALL CONTROL

/*
########################################
##############FUNCTIONS#################
########################################
*/

template <typename... Args>
void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., "\r\n");
    LTE.flush();
   delay(LTE_YIELD_MS);
  }
template <typename T>
void streamWrite(T last) {
    LTE.print(last);
  }
template <typename T, typename... Args>
void streamWrite(T head, Args... tail) {
    LTE.print(head);
    streamWrite(tail...);
  }
int8_t waitResponse(uint32_t timeout_ms, String& data, const char* c1, const char* c2) {
   data.reserve(64);
   uint8_t  index       = 0;
   uint32_t startMillis = millis();
   do {
      delay(LTE_YIELD_MS);
      while (LTE.available() > 0) {
        delay(LTE_YIELD_MS);
        int8_t a = LTE.read();
        if (a <= 0) continue;  
        data += static_cast<char>(a);
        //Serial.print("Data c1  -->");Serial.println(data);
        if (c1 && data.endsWith(c1)) {
          /*DEGUG*/
          // Serial.println("");
          // Serial.print("Data c1 ok >>>");Serial.println(data);
          // Serial.println("");
          /*END DEBUG*/
          index = 1;
          goto finish;
        } else if (c2 && data.endsWith(c2)) {
            index = 2;
            goto finish;
        } 
      }
    } while (millis() - startMillis < timeout_ms);
  finish:
    if (!index) {
      data.trim();
      if (data.length()) {  }
      /*DEGUG*/
      // Serial.println("");
      // Serial.print( "Data bug >>>");Serial.println(data);
      // Serial.println("");
      /*END DEBUG*/
      
      data = "";
    }
    return index;
}
int8_t waitResponse(uint32_t timeout_ms, const char* c1, const char* c2) {
  String data;
  return waitResponse(timeout_ms, data, c1, c2);
}
int8_t waitResponse( const char* c1, const char* c2) {
  return waitResponse(1000, c1, c2);
}
inline int16_t streamGetIntBefore(char lastChar) {
    char   buffer[7];
    size_t bytesRead = LTE.readBytesUntil(lastChar, buffer, static_cast<size_t>(7));
    if (bytesRead && bytesRead < 7) {
      buffer[bytesRead] = '\0';
      int16_t res    = atoi(buffer);
      return res;
    }
    return -9999;
  }
inline bool streamSkipUntil(const char c, const uint32_t timeout_ms) {
    uint32_t startMillis = millis();
    while (millis() - startMillis < timeout_ms) {
      while (millis() - startMillis < timeout_ms &&
             !LTE.available()) {
        delay(LTE_YIELD_MS);
      }
      if (LTE.read() == c) { return true; }
    }
    return false;
  }