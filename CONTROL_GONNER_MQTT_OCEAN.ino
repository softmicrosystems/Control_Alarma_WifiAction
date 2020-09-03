
//version modulos mios!!


//https://pubsubclient.knolleary.net/api
//https://github.com/knolleary/pubsubclient/issues/86

//solicitada p//si cuando da tension ya esta registrado en la red se enciende un led fijo (en ewelink ocupa el canal 1)
//mientras espera codigo, titila 3 veces con un separacion entre los destellos
//cuando aprendi-> da 5 beeps cortos
//Cuando no tiene ningun usuario debe oscila led estado LENTO
// Cuando tiene pore lo menos un usario, da piquiutos cada 5 segundos

//Cuando no tiene datos de red es la oscilacion de siempre
//Cuando no tiene internet -> es oscilacion rapida
//Cuando pulsa de5 svaegundos boton (Programacion) borra a default la red
//Cuando pulsa 10 segundos borra a default las programaciones

//Asegurae en mi plaqueta el led indicador con las polaridades, despuews adaptar a gonner

//A0:20:A6:12:58:15defaux
//131ab0sde
//3bd01dcac
//modulos actualesactmobu
//3bd1c1c
//43569rintentan
//125815
//3bcf3an
//1310b1

//https://programarfacil.com/esp8266/domotica-sonoff-wifi-espurna/https://programarfacil.com/esp8266/domotica-sonoff-wifi-espurna/
//https://www.itead.cc/sonoff-rf.html
//agregado de tx
//"modelo/idubicacion/IDModulo/TipoModulo/CanalRX/NivelSalida")
//y determina un 0 y 1 para apagar o encender salida
//como puede haber varias salida para un mismo receptor, se agrega salida de receptor (4 canales)
// la determinacion de como llegar a ese ubicacion se programa en el telefono mediante los graficos de tab/grupo/
//1- Ingresar tipos de modelo (automatico por el tipo de app android) -> Gon_PMG_AD
//2- Ubicacion geografica del modulo (casa o depto_ calle_piso_depto) -> Francia xxxx
//3- Id.Modulo    -> 125815
//4- Id.Receptor  -> 1,2 hasta 16
//5- Tipo Modulo  ->  0= Modulo alarma/PGM
//                    1= Receptor 2 o 4 canales
//                    2= salida PWM/LUZ

//5- CanalRx      -> caso Tipo=0 solo es PGM1 0 o 1
//                -> caso tipo=1 será 1,2  o 1,2,3,4 segun cual es la salida del receptor selecciondo
//                -> caso tipo=2 Cual de la lamparas sera controlado en intensidad (1 a 4)
//
//6- Nivel Salida -> 0 o 1  NOTA= si es una salida PWM (control luz el valor)S

//https://iotbyhvm.ooo/arduino-pubsubclient-arduino-client-for-mqtt/
// fcm.googleapis.com/fcm/send
//https://www.grc.com/fingerprints.htm
//https://github.com/francibm97/UM3750
//https://drive.google.com/drive/folders/0BzqRbfG5oG5XcXRTZzB2WmpzM3c

//https://techtutorialsx.com/2017/01/21/esp8266-watchdog-functions/
//https://github.com/FirebaseExtended/firebase-arduino/pull/401/commits/3530f7de36740a85dffe6f85333fbc91cbfacbfc

//http.addHeader("Authorization", "key=AAAAtA3ovNE:APA91bH8SKLfgIzslq_AGpT6NP-oJocxxp71LHv0-85y87gl0wpSd2iVfe6Nq5IrcnCXYB4cXTHR8kOiRfc_oCpT937pRK58rCG_KHMmI_gKlaPasJG8UhMoSyIuZYgz7yGhNsVIJGSX");
//https://dzone.com/articles/iot-push-notifications-arduino
//http://www.bujarra.com/poniendo-tasmota-en-un-sonoff-con-una-raspberry-pi/

//https://github.com/arendst/Sonoff-Tasmota/wiki/Commands

//http://www.bujarra.com/usando-un-lector-de-huellas-dactilares-en-raspberry-pi/#more-22425

//https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/

//https://firebase.google.com/docs/reference/fcm/rest/v1/projects.messages#androidnotification

//35956 3081245999    Motorola
// 5060141652    Lg viejo
// 3090066175    Lg nuevo?r
// 4098062429    quantumtesta

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#else
#include <WiFi.h>          //https://github.com/esp8266/Arduino
#endif

//#include <RCSwitch.h>
//#include <math.h>

//needed for librarys
//#include <DNSServer.h>

#if defined(ESP8266)
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiClient.h>
#else
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager
#endif

#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
//#include <Bounce2.h>
#include <PubSubClient.h>

//______________________________________________________
int PinInAux = 12;     //D6   GPIO12  MISO

#define FIREBASE_HOST "controlalarmasgonner.firebaseio.com"
//#define FIREBASE_AUTH "Ycv2tS2Quf9xLqwizWnNpSbvtCaiOiYU5nfMGLAMO" //125815

static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";      //OK
const int timeZone = -3; // Central European Time

//________________________________
//ENTRADAS DE BORNERA  BORNERA NODEMCU
int PinArmDesarm    = 2;      //D4  GPIO2  TXD1     SE USA COMO LED INTERNO EN PLQUETA Con "1" el led se apaga}
//                    FALLA BOOT SI SE LO PONEA A MASA
int PinInSir        = 14;     //D5  GPIO14  SCLK
int PinLedVerde     = 15;     //D8  GPIO15  CS/TXD2
int PinInPanico     = 13;     //D7  GPIO13  MOSI/RXD2

// SALIDAS DE BORNERAs lee
int PinOut1Pgm  = 0;          //D3= GPIO0   WAKE    Falla BUTEO si pone entrada a MASA
int PinOut2Pgm  = 4;          //D2= GPIO4   SDA
int PinOutRemoto = 5;         //D1= GPIO5   SCL
int PinLedAmarillo = 16;      //D0= GPIO16  WAKE    //Normalmente a "1" al boot!

//NO USAR COMO SALIDA DE RELAY GPIO1,GPIO3,GPIO9,GPIO10,GPIO16


/*
  //_______________________________________
  CONEXIONADO PINOUT MODULO GONNER WRCOM2
      VCC            GND
      EN             IO10 (IN A/D)
  PGM2    IO14           TOUT
  PGM1    IO12           RST
  OUTA/D  IO13           IO5  (IN_PANICO)
  LED1    IO15           GND
  LED2    IO02           TX
  pulsado IO00           RXF
      GND            IO4 (IN_SIR)
*/


/*
  //__________________________________________________________
  //  IO PARA MODULO WRCOM2  GONNER
  int PinArmDesarm=16;   // ENTRADA=DETECTA ARM/DESRAMA EXTERNO
  int PinInSir=4;    //Sensado de sirena externaa
  int PinLedAmarillo=2;    // led status 1
  int PinInPanico=5;  //sensado de panico de bornera

  int PinOut1Pgm=12;    //Salida PGM1
  int PinOut2Pgm=14;    //Salida PGM2
  int PinOutRemoto=13;  //salida a eauipo externo para camviar estado
  int PinLedAmarillo=15;     //Led status 2

*/

//Bounce debouncer1 = Bounce();
//_________________
//Posiciones eeprom de parametros a usar
const int E2_IDReporte1 = 00;
const int E2_IDReporte2 = 10;
const int E2_IDReporte3 = 20;
const int E2_IDReporte4 = 30;
const int E2_IDReporte5 = 40;
const int E2_IDReporte6 = 50;
const int E2_IDReporte7 = 60;
const int E2_IDReporte8 = 70;
//_____
const int E2_UbicacionID1 = 80;
const int E2_UbicacionID2 = 100;
const int E2_UbicacionID3 = 120;
const int E2_UbicacionID4 = 140;
const int E2_UbicacionID5 = 160;
const int E2_UbicacionID6 = 180;
const int E2_UbicacionID7 = 200;
const int E2_UbicacionID8 = 220;
//________
const int E2_UbicacionMod = 240;
const int E2_Clave_Admin = 260;
const int E2_Clave_Usua = 264;
const int E2PosTimHb = 268;
//_____


const int E2_Ruteo_ID1 = 270;
const int E2_Ruteo_ID2 = 271;
const int E2_Ruteo_ID3 = 272;
const int E2_Ruteo_ID4 = 273;
const int E2_Ruteo_ID5 = 274;
const int E2_Ruteo_ID6 = 275;
const int E2_Ruteo_ID7 = 276;
const int E2_Ruteo_ID8 = 277;

const int E2SalidaAD = 278; //Modo de salida A/D pulso / nivel
const int E2TimPGM1 = 279;
const int E2TimPGM2 = 280;
const int E2TimPulAD = 281;
const int E2ModoPGM1 = 282;
const int E2ModoPGM2 = 283;
const int E2_TIPO_TIM1 = 284;
const int E2_TIPO_TIM2 = 285;
const int E2_ST_ARMED = 286;

const int InicE2 = 288;
//_________________________________________
//12577DLMNJVS
const int E2_TIMER1 = E2_ST_ARMED + 1;
// y= dia de la semana
// x es       01=on pgm1 /02=off pgm1
//            03=ON PGM2 /04=OFF PGM2
//            05=ARMED    /10=ARMED
//            06=ARMED    /10=DISARMED
const int E2_TIMER2 = E2_TIMER1 + 12;
const int E2_TIMER3 = E2_TIMER2 + 12;
const int E2_TIMER4 = E2_TIMER3 + 12;
const int E2_TIMER5 = E2_TIMER4 + 12;
const int E2_TIMER6 = E2_TIMER5 + 12;
const int E2_TIMER7 = E2_TIMER6 + 12;
const int E2_TIMER8 = E2_TIMER7 + 12;

const int E2_CON_INTERNET = E2_TIMER8 + 12;
const int E2_CLV_ADMIN    = E2_CON_INTERNET + 1;
const int E2_BORRA_RED    = E2_CLV_ADMIN + 4;
const int E2_NEXT         = E2_BORRA_RED + 1;

//const char* topic = "someTopic";
String clientId;      // = "randomClientId";

String ValUbicacionMod;
String UsuarioGenerador;
String Titulo;
String Descripcion;
String TituloEvento = "Evento en " + ValUbicacionMod;   //"Novedad desde dispositivo ";
String TextoNovedad = Descripcion;
int count;
int n = 0;
String temporal;
String MensajeEnviar;
String GeneradorPublicacion;
String HoraFechaActual;
String string_variable;
String ValorBuscado;
String IdModuloDestinatario;
String ImeiAutorizado;
String DirSubscribe;

byte TotalLeds;

//byte DebeRepetirEvento;
byte posicion;
byte ValModoOutAD;
byte ValTimOutAD;
byte HayInternet;
byte HayDeteccionAux;
byte HayUsuario1, HayUsuario2, HayUsuario3, HayUsuario4, HayUsuario5, HayUsuario6, HayUsuario7, HayUsuario8;
byte YaContol5SegRstSSID;
byte YaContoTiempoDefault;

byte SyncLedAmarillo;
byte cntFaltaInternet;
//byte NoHayConexionMQTT;
byte ExisteUsuario;
String NombreUsuaEmisor;
byte StatusIn;

//_____________
byte ValRutId1, ValRutId2, ValRutId3, ValRutId4, ValRutId5, ValRutId6, ValRutId7, ValRutId8;

int tempo;
byte MuestraCuentaMinutos;
byte CoincideDiaSemanaActual;
byte EsAccionLocal;
byte DebeContarTiempoDefault;       //una vez que pulse el boton de AUX empieza a contar tiempo para determinar DEFAULT o tiempo SSID red
byte CntTimDefault;
byte DebeDesactivarSalida1;
byte DesactivaPorTiempoPGM1;
byte DebeDesactivarSalida2;
byte DesactivaPorTiempoPGM2;
byte AvisarCambioIDUsuarios;

byte ValTimPGM1;
byte ValTimPGM2;
byte ValModoPGM1;
byte ValModoPGM2;
byte ValModoCuentaPGM1;
byte ValModoCuentaPGM2;
byte EsModoShare;
byte YaInicializoEEprom;
byte Respuesta;

byte EstaArmado;        //00=Desarmado 01=ARMADO
byte StSir;

byte DebeResponderAOrigen;
byte DebeEnviarNotificacionesSegunRuteo;

int CntTimOut1; // tiwmpo para control de PGM1 (sea luz o sirena)???
int CntTimOut2;

byte LedTest;
byte HayDeteccionPanico;
byte DebeOscErrStatus;
byte HayAvisoFaltaInternet;
bool Start = false;

byte StPGM1;
byte StPGM2;


//________________________________________
String UsuarioEmisor;
String valCompara;
String DiaFiltrado;
String OrigenTel;
String ValTim1, ValTim2, ValTim3, ValTim4, ValTim5, ValTim6, ValTim7, ValTim8;
String RamHHTim1, RamMMTim1, RamSelDias1, RamModoAct1;
String RamHHTim2, RamMMTim2, RamSelDias2, RamModoAct2;
String RamHHTim3, RamMMTim3, RamSelDias3, RamModoAct3;
String RamHHTim4, RamMMTim4, RamSelDias4, RamModoAct4;
String RamHHTim5, RamMMTim5, RamSelDias5, RamModoAct5;
String RamHHTim6, RamMMTim6, RamSelDias6, RamModoAct6;
String RamHHTim7, RamMMTim7, RamSelDias7, RamModoAct7;
String RamHHTim8, RamMMTim8, RamSelDias8, RamModoAct8;

String RamDiaActual, RamMesActual, RamAnoActual, RamHoraActual, RamMinuteActual, RamDiaSemanaActual;

int a;
int b;
int c;
int d;
unsigned long timeLastCheck = 0;
unsigned long intervalCheck = 4000;

//_____
//String UbicacionIdUsuarioProgramador;
String DirPublicacion;

String IDOrigenEnvio;
String stringOne;
String ValIdReporte1, ValIdReporte2, ValIdReporte3, ValIdReporte4, ValIdReporte5, ValIdReporte6, ValIdReporte7, ValIdReporte8;
String ValUbicacionID1, ValUbicacionID2, ValUbicacionID3, ValUbicacionID4, ValUbicacionID5, ValUbicacionID6, ValUbicacionID7, ValUbicacionID8;
//String ValUbicacionMod;
String DatoRecibido;
String resultado1;
String IdImeiUsuarioNuevo;
String NombreUsuarioNuevo;
String TxtUbicaModulo;

String DireccionOrigen;
String ValClaveAdmin;
String ValClaveUsua;
String ModuloTest;
String ComandoActualizaValores;
String RamValClvAdmin;

/*
  const char* mqttServer = "192.168.0.118";
  const char* mqttUser =  "SoftmicroGonner";
  const char* mqttPass = "alejandro";
  const int mqttPort = 8083;
*/


//const char* mqttServer = "Gonner@broker.emqx.io";
//const char* mqttUser = "softmicro";
//const char* mqttPass = "marita64";
//const char* mqttServer = "ioticos.org";
//const char* mqttUser = "MuCaxo96Gx7MK7k";
//const char* mqttPass = "jp2i3ued0J355KN";

//const char* root_topic_subscribe = "GN8rYIiMtH9Tt89";
//const char* root_topic_publish = "GN8rYIiMtH9Tt89/output";



const int mqttPort = 1883;
const char* mqttServer = "167.71.119.247";
const char* mqttUser = "tecnosoftmicro";
const char* mqttPass = "TecnoMicro54Mqtt";

// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
//#define MQTT_PORT 1883
//#define MQTT_USERNAME "tecnosoftmicro"
//#define MQTT_PASSWORD "TecnoMicro54Mqtt"



/*
  //____________________
  const char* mqttServer = "visualnet.ddns.me";
  const char* mqttUser =  "Alejandro";
  const char* mqttPass = "MisTestes2020";
  //const int mqttPort = 1883;

  ¨/*
  const char* mqttServer = "node02.myqtthub.com";
  const char* mqttUser =  "softmicro";
  const char* mqttPass = "r3lTKpI6-CXV8r9j8";
  const int mqttPort = 1883;
*/

//"3090066175";

/*
  const char* mqttServer = "m13.cloudmqtt.com";
  const char* mqttUser =  "oqwipvej";
  const char* mqttPass = "4GTy6l3B-rjN";
  const int mqttPort = 11480;
*/

/*
  const char* mqttServer ="m10.cloudmqtt.com";
  const char* mqttUser =  "ykirliwk";
  const char* mqttPass = "6iTBa9PeEk1P";    //es un UNO y no una ele
  const int mqttPort = 14079;
*/

/*
  const char* mqttServer = "postman.cloudmqtt.com";
  const char* mqttUser =  "ndijnuei";
  const char* mqttPass = "HWHy4e_YBPQJ";    //es un UNO y no una ele
  const int mqttPort = 10317;
*/

/*
  const char *mqtt_server = "tecnosoftmicro.ga";
  const int mqtt_port = 1883;
  const char *mqtt_user = "web_client";
  const char *mqtt_pass = "121212";
*/



//___________________________________________
WiFiClient espClient;
PubSubClient MQTTClient(espClient);
String IdOrigen;

int value;
int pos;
//_____
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 3000;

unsigned long startMillis1;
unsigned long currentMillis1;
const unsigned long period1 = 60000;
//___________________
int CntContMs;
int CntSegundos;
int CntMinutos;
//___
int CuentaSegundosPGM1; //permite contar tiempo en segundos
int CuentaMinutosPGM1;  //idem minutos
int CuentaHorasPGM1;    // idem horas
//___
int CuentaSegundosPGM2;
int CuentaMinutosPGM2;
int CuentaHorasPGM2;

//________
int CntSegundosPGM1; //Cuenta segundos por el lado de PGM 1
int CntSegundosPGM2; //Idem para PGM2
//___
int CntMinutosPGM1;
int CntMinutosPGM2;

int CntHorasPGM1;
int CntHorasPGM2;

String ValIn1;
String resultado;

String variable;
String recibido;
String line;
String url1;
String NumSerialESP;


int YaDetectoEntradaSirena = 0;
int YaEnvioCambioPGM1 = 0;
byte EnvioDetArmado;
String url;
//String Origen= "" + IdOrigen + "";

String IdDestino;
byte estadetectandoNoConnect;

WiFiUDP Udp;
unsigned int localPort = 8888; // local port to listen for UDP packets

//Serial.println("lee y muestra la hora");
time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

Ticker tickerSSID;
Ticker blinker;

int address;
int posicionWr;
int DatoE2;
String CmdBasico = "&from=" + IdOrigen + "&to=" + IdDestino + "&data=";
String CmdAccion;
String TopicoDestino;

//_____________________
void setup() {
  Serial.begin(115200);
  /*
    float resultado;
    float num1;
    float num2;

    num1 = 1.2;
    num2 = 0.98;
    resultado = num1 * num2;

    Serial.println(resultado);
  */
  pinMode(PinOut1Pgm, OUTPUT);
  pinMode(PinOut2Pgm, OUTPUT);
  pinMode(PinOutRemoto, OUTPUT);
  pinMode(PinLedVerde, OUTPUT);

  pinMode(PinArmDesarm, INPUT_PULLUP);    //_PULLUP);        //Armado/desarmado
  pinMode(PinInSir, INPUT_PULLUP);    // Alarma/Sirena
  pinMode(PinInPanico, INPUT_PULLUP); //Panico externo
  pinMode(PinLedAmarillo, OUTPUT);


  digitalWrite(PinOut1Pgm, LOW);
  digitalWrite(PinOut2Pgm, LOW);
  digitalWrite(PinOutRemoto, LOW);
  digitalWrite(PinLedAmarillo, LOW);

  digitalWrite(PinLedVerde, LOW);    //HIGH);

  //___________________
  //Local intialization. Once its business is done, there is no need to keep it around
  //start tickerSSID with 0.5 because we start in AP mode and try to connect
  tickerSSID.attach(0.6, tick);
  blinker.attach(0.1, Timers);
  WiFiManager wifiManager;
  wifiManager.setTimeout(120);
  wifiManager.setBreakAfterConfig(true);


  //wifiManager.resetSettings();

  NumSerialESP = String(ESP.getChipId(), HEX);
  if (NumSerialESP.length() < 6 ) {
    NumSerialESP = "0" + NumSerialESP;
  }

  Serial.println(NumSerialESP);
  String NumSerialESP1 = String(ESP.getChipId(), DEC);
  Serial.println(NumSerialESP1);

  wifiManager.autoConnect();
  Serial.println("__________________");
  if (!wifiManager.autoConnect())  {
    Serial.println("Fallo conexion, debera resetear para ver si conecta");
    tickerSSID.detach();
    delay(3000);
    for (int i = 0; i < 5; i++) {
      digitalWrite(PinLedAmarillo, LOW); //led verde
      delay(50);
      digitalWrite(PinLedAmarillo, HIGH);
      delay(50);
    }
    Serial.println("Va a resetear por falta de red!");
    ESP.reset();
    delay(2000);
  }

  Serial.println("__________________");
  //if you get here you have connected to the WiFi
  Serial.println("Conectado a la red :)");
  tickerSSID.detach();
  //keep LED on
  digitalWrite(PinLedAmarillo, LOW);
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());


  EEPROM.begin(2048);
  //debouncer1.attach(PinArmDesarm);
  //debouncer1.interval(500);

  // ESP.wdtDisable();   ???????????????????????????????????
  ESP.wdtEnable(5000);

  //________
  address = InicE2;
  RdE2();
  YaInicializoEEprom = Respuesta;

  if (YaInicializoEEprom != 53)  {
    //BorraE2();
    CargaDefault();
    DatoE2 = 53;
    address = InicE2;
    WrE2();
    //cleansession=false
    //Serial.println("Grabacion completa");
  }

  digitalWrite(PinLedAmarillo, HIGH);
  LeeModoSalidaRemoto();
  address = E2_ST_ARMED;
  RdE2();
  if (Respuesta == 1) {
    EstaArmado = 1;        //"SISTEMA ARMADO";
  } else {
    Serial.println("Ultimo estado = DESARMADO");
    EstaArmado = 0;       //SISTEMA DESARMADO
  }


  //_______________________
  Udp.begin(localPort);
  //Serial.print("Local port: ");
  //Serial.println(Udp.localPort());
  ///Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  digitalClockDisplay(); //Lee y muestra hORA Y FECHA

  //ESP.wdtDisable();
  //ESP.wdtEnable(5000);
  GetExternalIP();
  delay(3000);

  /*
    mySwitch = RCSwitch();
    mySwitch.enableTransmit(16);
    mySwitch.setPulseLength(1);
    mySwitch.setProtocol(6);
    mySwitch.setRepeatTransmit(15);
  */
  MQTTClient.setServer(mqttServer, mqttPort);
  MQTTClient.setCallback(callback);
  while (!MQTTClient.connected()) {

    //cleansession

    if (MQTTClient.connect("ESP8266Client", mqttUser, mqttPass)) {
      Serial.println("SE HA CONECTADO EL MQTT");
      Serial.println("______________________");
      Serial.println(" ");

      temporal = "PROGMODULO";
      MQTTClient.subscribe("PROGMODULO");   //String(temporal).c_str());   //subscripcion universal!
      Serial.println("Acaba de subscribirse a PROGRMODULO!");
      //MQTTClient.subscribe("125815/home/office/");      //|desk");
      LeeDatosMemoria();

      //DirSubscribe = "GN8rYIiMtH9Tt89/";
      DirSubscribe = String(NumSerialESP).c_str();
      //DirSubscribe= "GN8rYIiMtH9Tt89/125815";
      MQTTClient.subscribe(String(DirSubscribe).c_str());
      Serial.println("Subscribe a : " + DirSubscribe);
      Serial.println("_____________________");


    } else {
      Serial.print("failed with state ");
      Serial.print(MQTTClient.state());
      //NoHayConexionMQTT = 1;
      return;
    }
  }
}

//________________________________________
//IMPORTANTE
/*
  LeeTimer1();
  LeeTimer2();
  LeeTimer3();
  LeeTimer4();
  LeeTimer5();
  LeeTimer6();
  LeeTimer7();
  LeeTimer8();

  Serial.println(RamHHTim1 + ":" + RamMMTim1 + "-" + RamModoAct1 + "-" + RamSelDias1);
  Serial.println(RamHHTim2 + ":" + RamMMTim2 + "-" + RamModoAct2 + "-" + RamSelDias2);
  Serial.println(RamHHTim3 + ":" + RamMMTim3 + "-" + RamModoAct3 + "-" + RamSelDias3);
  Serial.println(RamHHTim4 + ":" + RamMMTim4 + "-" + RamModoAct4 + "-" + RamSelDias4);
  Serial.println(RamHHTim5 + ":" + RamMMTim5 + "-" + RamModoAct5 + "-" + RamSelDias5);
  Serial.println(RamHHTim6 + ":" + RamMMTim6 + "-" + RamModoAct6 + "-" + RamSelDias6);
  Serial.println(RamHHTim7 + ":" + RamMMTim7 + "-" + RamModoAct7 + "-" + RamSelDias7);
  Serial.println(RamHHTim8 + ":" + RamMMTim8 + "-" + RamModoAct8 + "-" + RamSelDias8);
*/
//_________________________
time_t prevDisplay = 0; // when the digital clock was displayed

//________________________________________
void CargaDefault() {
  //Serial.println("_______________________");
  Serial.println("Grabando default de eeprom");
  //____________
  //DatoE2=4;   //10;
  //address=E2PosTimHb;
  //WrE2();
  //__________________
  address = E2_Clave_Admin;
  string_variable = "1234";
  GrabaPaqueteE2();
  //__________________
  address = E2_Clave_Usua;
  string_variable = "1111";
  GrabaPaqueteE2();
  //__________________
  address = E2_CLV_ADMIN;
  string_variable = "1111";
  GrabaPaqueteE2();

  //__________________
  CargaDefaultUbicacion();
  LeeUbicacionModulo();

  //____________-
  // guarda default de datos de USUARIO (ID y UBICACION)
  Carga_Default_ID_Usuario1();
  BorraE2promNombreUsuario1();

  //___
  Carga_Default_ID_Usuario2();
  BorraE2promNombreUsuario2();
  //___
  Carga_Default_ID_Usuario3();
  BorraE2promNombreUsuario3();
  //___
  Carga_Default_ID_Usuario4();
  BorraE2promNombreUsuario4();
  //___
  Carga_Default_ID_Usuario5();
  BorraE2promNombreUsuario5();
  //___
  Carga_Default_ID_Usuario6();
  BorraE2promNombreUsuario6();
  //___
  Carga_Default_ID_Usuario7();
  BorraE2promNombreUsuario7();
  //___
  Carga_Default_ID_Usuario8();
  BorraE2promNombreUsuario8();

  //____________
  DatoE2 = 2;
  address = E2ModoPGM1;
  WrE2();

  //____________
  DatoE2 = 2;
  address = E2ModoPGM2;
  WrE2();
  //____________
  DatoE2 = 1;
  address = E2TimPGM1;
  WrE2();

  DatoE2 = 1;
  address = E2TimPGM2;
  WrE2();
  //__
  DatoE2 = 3;
  address = E2TimPulAD; //tiempo del pulso de armado/desarmado
  WrE2();

  //____________
  DatoE2 = 2; //minutos
  address = E2_TIPO_TIM1;
  WrE2();
  //____________         HA
  DatoE2 = 2; //minutos
  address = E2_TIPO_TIM2;
  WrE2();

  //__________
  DatoE2 = 1;
  address = E2SalidaAD;   // 00=Modo nivel
  //01=modo PULSOso de determinada duracion
  WrE2();

  //_______
  DatoE2 = 1;             //es desarmado por default!
  address = E2_ST_ARMED;
  WrE2();

  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID1;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID2;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID3;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID4;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID5;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID6;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID7;
  WrE2();
  //____________
  DatoE2 = 15;
  address = E2_Ruteo_ID8;
  WrE2();
  //____________
  CargaDefTimer1(); //simula e2 donde se cargan datos los datos de tiempos
  CargaDefTimer2();
  CargaDefTimer3();
  CargaDefTimer4();
  CargaDefTimer5();
  CargaDefTimer6();
  CargaDefTimer7();
  CargaDefTimer8();
}

//_____________________      -
void tick() {
  //toggle state
  int state = digitalRead(PinLedAmarillo); // get the current state of GPIO1 pin
  digitalWrite(PinLedAmarillo, !state);    // set pin to the opposite state
}

//______xgets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());

  //entered config mode, make led toggle faster
  tickerSSID.attach(0.4, tick);
}


//________________________________________________________________________
void loop() {
  ChequeaEnvioPaquete();
  if (HayInternet == 1)  {
    AnalizaStringRecibido();
  }

  //___________
  if (MuestraCuentaMinutos == 1)  {
    MuestraCuentaMinutos = 0;
    time_t t;
    //ojo que si lee 1970-> no cambioe NOW!!!!
    ValorBuscado = "1970";
    pos = HoraFechaActual.indexOf(ValorBuscado);
    if (pos >= 0)    {
      //Serial.println("existe el 1970 no debe actualizar el relohj interno");
      setSyncProvider(getNtpTime);
      setSyncInterval(300);
      //Serial.println("va a leer cada minuto?");
      digitalClockDisplay();
    } else {
      t = now();
    }
    //____________________________________
    //time_t t = now();       //????????????????????
    RamDiaActual = day(t);
    tempo = RamDiaActual.toInt();
    if (tempo < 10) {
      RamDiaActual = '0' + RamDiaActual;
    }
    //Serial.print(RamDiaActual);
    //Serial.print(+"/");

    RamMesActual = month(t);
    tempo = RamMesActual.toInt();
    if (tempo < 10)    {
      RamMesActual = '0' + RamMesActual;
    }
    RamHoraActual = hour(t);
    tempo = RamHoraActual.toInt();
    if (tempo < 10) {
      RamHoraActual = '0' + RamHoraActual;
    }

    //Serial.print(RamHoraActual);
    //Serial.print(+":");
    RamMinuteActual = minute(t);
    tempo = RamMinuteActual.toInt();
    if (tempo < 10) {
      RamMinuteActual = '0' + RamMinuteActual;
    }
    //Serial.println(RamMinuteActual);
    RamAnoActual = year(t);
    RamDiaSemanaActual = weekday(t); //numero de 1 a 8 que indica el dia del la fecha leida
    tempo = RamDiaSemanaActual.toInt();

    //____________________________________________________________________________________________
    LeeTimer1();
    resultado = RamSelDias1;
    ArmaFiltroDiaSemana();
    //___________
    //Analiza posibilidades de hora/fecha y dia
    a = RamSelDias1.toInt();
    if (CoincideDiaSemanaActual == 1) {
      // Hay uno o mas dias que deben cumplir condicion no por el valor el dia y el mes sino por el dia de la semana!!
      if (RamHoraActual == RamHHTim1 && RamMinuteActual == RamMMTim1)      {
        tempo = RamModoAct1.toInt();
        DefineTipoAccionxTim();
        //
        //Serial.println("aqui Cumple con hora sin importar la fecha porque si coincide el nombre del dias");
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }


    //________________________________________
    LeeTimer2();
    resultado = RamSelDias2;
    a = RamSelDias2.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1)  {
      if (RamHoraActual == RamHHTim2 && RamMinuteActual == RamMMTim2)      {
        tempo = RamModoAct2.toInt();
        DefineTipoAccionxTim();
        //Serial.println("Coincide TIEMPO 2:");
        // Serial.println("aqui Cumple con hora sin importar la fecha porque si coincide el nombre del dias");
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }

    //________________________________
    LeeTimer3();
    resultado = RamSelDias3;
    a = RamSelDias3.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1) {
      if (RamHoraActual == RamHHTim3 && RamMinuteActual == RamMMTim3)      {
        tempo = RamModoAct3.toInt();
        DefineTipoAccionxTim();
        //Serial.println("Coincide TIEMPO 3:");
        //Serial.println("aqui Cumple con hora sin importar la fecha porque si coincide el nombre del dias");
      } else  {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }

    //____________________________
    LeeTimer4();
    resultado = RamSelDias4;
    a = RamSelDias4.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1) {
      // Hay uno o mas dias que deben cumplir condicion no por el valor el dia y el mes sino por el dia de la semana!!
      if (RamHoraActual == RamHHTim4 && RamMinuteActual == RamMMTim4) {
        tempo = RamModoAct4.toInt();
        DefineTipoAccionxTim();
        //Serial.println("Coincide TIEMPO 4:");
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }
    LeeTimer5();
    resultado = RamSelDias5;
    a = RamSelDias5.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1)  {
      // Hay uno o mas dias que deben cumplir condicion no por el valor el dia y el mes sino por el dia de la semana!!
      if (RamHoraActual == RamHHTim5 && RamMinuteActual == RamMMTim5) {
        //tempo = RamModoAct5.toInt();
        DefineTipoAccionxTim();
        //Serial.println("Coincide TIEMPO 5:");
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }

    //______________________________________________________
    /*
      string_variable = "16056DLMNJVS"; //activa PGM 1 todos los dias a la misma hora (incluidos domingos
      address = E2_TIMER6;
      GrabaPaqueteE2();
    */
    LeeTimer6();
    resultado = RamSelDias6;
    a = RamSelDias6.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1) {
      // Hay uno o mas dias que deben cumplir condicion no por el valor el dia y el mes sino por el dia de la semana!!
      if (RamHoraActual == RamHHTim6 && RamMinuteActual == RamMMTim6) {
        tempo = RamModoAct6.toInt();
        DefineTipoAccionxTim();
        //Serial.println("Coincide TIEMPO 6:");
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }

    //________
    LeeTimer7();
    resultado = RamSelDias7;
    a = RamSelDias7.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1) {
      // Hay uno o mas dias que deben cumplir condicion no por el valor el dia y el mes sino por el dia de la semana!!
      if (RamHoraActual == RamHHTim7 && RamMinuteActual == RamMMTim7) {
        tempo = RamModoAct7.toInt();
        //Serial.println("Coincide TIEMPO 7:");
        DefineTipoAccionxTim();
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }

    //______________________
    /*
      string_variable = "16074DLMNJVS"; //activa PGM 1 todos los dias a la misma hora (incluidos domingos
      address = E2_TIMER8;
      GrabaPaqueteE2();
    */
    //LeeTimer8();
    /*
      Serial.println(RamHHTim8);
      Serial.println(RamMMTim8);
      Serial.println(RamSelDias8);
      Serial.println(RamModoAct8);
    */
    resultado = RamSelDias8;
    a = RamSelDias8.toInt();
    ArmaFiltroDiaSemana();
    if (CoincideDiaSemanaActual == 1) {
      if (RamHoraActual == RamHHTim8 && RamMinuteActual == RamMMTim8) {
        tempo = RamModoAct8.toInt();
        // Serial.println("Coincide TIEMPO 8:");
        DefineTipoAccionxTim();
      } else {
        //Serial.println("No coincide el horario a pesar que es un dia existente entre los posibles a analizar");
      }
    }
  }

  //_________________________________
  currentMillis1 = millis(); //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis1 - startMillis1 >= period1) {
    startMillis1 = currentMillis1;
    GetExternalIP();
    setSyncProvider(getNtpTime);
    setSyncInterval(300);
    digitalClockDisplay();
  }

  currentMillis = millis(); //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  {
    startMillis = currentMillis;
  }

  LeeEntradaArmar(); // lee nivel entrada de armado/desarmado
  LeeStSirena();     //lee estado de sirena
  LeePanico();       // lee boton Panico
  LeeInAuxiliar();

  if (!MQTTClient.connected()) {
    reconnect();
  }
  MQTTClient.loop();
}


//___________________________
void OscilaLedStatus() {
  if (DebeOscErrStatus == 1)  {
    if (LedTest == 1)    {
      digitalWrite(PinLedAmarillo, LOW); //apaga led
      LedTest = 0;
    } else {
      digitalWrite(PinLedAmarillo, HIGH); //Activa leg
      LedTest = 1;
    }
  }
}

//____
void LeeEntradaArmar() {
  address = E2_ST_ARMED;
  RdE2();
  if (Respuesta == 1) {
    EstaArmado = 1;    //Ultimo estado = ARMADO
  } else {
    EstaArmado = 0;   //ultimo estado DESARMADO
  }
  if (digitalRead(PinArmDesarm) == 0) {          // Detecta entra a masa (ojo que la SIRENA puede ser una MASA o un ALTO
    //Entrada a masa = SISTEMA ARMADO
    if (EstaArmado == 0) {      //Si previamnete estaba desarmado->
      delay(100);
      if (digitalRead(PinArmDesarm) == 0) {
        //si se mantiene en nivel MASA-> Confirmado que va a ARMAR si no hubo armado previo
        EstaArmado = 1;
        Serial.println("SISTEMA ARMADO");
        DatoE2 = 1;
        address = E2_ST_ARMED;
        WrE2();
        CmdAccion = "Sistema armado";
        Serial.println("Envia comando al servidor de sistema ACTIVADO");
        Descripcion = CmdAccion;         //"El sistema se ha ARMADO";
        EsAccionLocal = 1;
        DebeEnviarNotificacionesSegunRuteo = 1;
      }
    }
  } else {
    //entrada digital esta en "1" porque esta abierta el boton de prueba
    //con nivel=1 deberia ser siempre DESARMADO
    if (EstaArmado == 1) {
      //si previamente estaba armado->
      delay(100);
      if (digitalRead(PinArmDesarm) == 1) {
        //Si confirmo que esta en "1" (DESARMADO"-> a desarmar
        EstaArmado = 0;         //ES desarmado
        DatoE2 = 0;
        address = E2_ST_ARMED;        //modo desarmado
        WrE2();
        Serial.println("");
        Serial.println("___________________");
        Serial.println("SISTEMA DESARMADO");
        CmdAccion = "Sistema desarmado";
        Serial.println("Envia comando al servidor de sistema DESACTIVADO");
        Descripcion = "desactivar";
        EsAccionLocal = 1;
        DebeEnviarNotificacionesSegunRuteo = 1;
      }
    }
  }
}

//_____________________________________
void LeeStSirena() {
  if (digitalRead(PinInSir) == 0) {
    // Detecta entra a masa (ojo que la SIRENA puede ser una MASA o un ALTO
    if (YaDetectoEntradaSirena == 0) {
      delay(2500);
      if (digitalRead(PinInSir) == 0) {
        YaDetectoEntradaSirena = 1;
        StSir = 1;
        Serial.println("SIRENA ACTIVADA");
        //aqui esta armado???

        if (EstaArmado == 1) {
          // se detecta sirena + esta armado-> EVENTO!
          CmdAccion = "alarma nueva";
          UsuarioGenerador = "Desde su alarma";
          EnviaNotificacionPush(NumSerialESP);
        } else {
          Serial.println("Activa sirena localmente sin aviso al usuario");
        }
      }
    }
  } else {
    if (YaDetectoEntradaSirena == 1) {
      delay(1500);
      if (digitalRead(PinInSir) == 1) {
        StSir = 0;
        YaDetectoEntradaSirena = 0;
        Serial.println("NO ENVIA  AVISO DE SIRENA DESACTIVADA");
        CmdAccion = "Sirena normalizada";
        // ojo deberia enviarlo?? DebeEnviarNotificacionesSegunRuteo = 1;
      }
    }
  }
}
//________________
void LeePanico() {
  if (digitalRead(PinInPanico) == 0)  {
    if (HayDeteccionPanico == 0) {
      delay(500);
      if (digitalRead(PinInPanico) == 0) {
        Serial.println("ATENCION! PANICO");
        HayDeteccionPanico = 1;
        CmdAccion = "PANICO LOCAL";
        Serial.println("Por pulsar boton de PANICO desde bornera");
        IDOrigenEnvio = "dispositivo";
        UsuarioGenerador = "Desde su alarma";
        UsuarioGenerador = UsuarioGenerador  + "," + ValUbicacionMod;
        EnviaNotificacionPush(NumSerialESP);
      }
    }
  } else {
    //hay nivel 1 -> solto boton!
    if (HayDeteccionPanico == 1) {
      delay(500);
      if (digitalRead(PinInPanico) == 1) {
        Serial.println("Normalizo boton de panico");
        HayDeteccionPanico = 0;
      }
    }
  }
}

//___________________________________________
void Timers() {
  //Serial.println("Pasaron 100 segundos");
  OscilaLedStatus();
  CntContMs = CntContMs + 1;
  if (CntContMs >= 10) {
    //si pasaron 10 lazosn de 100 ms -> es un segundo
    CntContMs = 0;
    CntSegundos++; //= CntSegundos+1;       //contado de segundos
    //Serial.println(CntSegundos);
    //Serial.print(".");
    if (CntSegundos == 60)    {
      CntSegundos = 0;
      //Serial.println(" Conto 1 minuto");
      MuestraCuentaMinutos = 1;
    }

    //____________
    if (DebeContarTiempoDefault != 0)  {
      //Esta contando tiemnpoo de DEFAULT/RED
      // if (YaContoTiempoDefault == 0) {
      //Puede contar tiempo porque aun no llego a default!
      //if (CntTimDefault != 0)      {
      CntTimDefault++;      // = CntTimDefault + 1; //
      if (CntTimDefault <6) {
        Serial.println("Cuenta tiempo para borrar SSID primero:");
      }else {
         Serial.println("Ahora cuenta para generar DEFAULT!");
      }
      Serial.println(CntTimDefault);
      //para hacer togle en cada acceso
      if (YaContol5SegRstSSID==0) {
        int state1 = digitalRead(PinLedVerde); // get the current state of GPIO1 pin
        digitalWrite(PinLedVerde, !state1);
        if (CntTimDefault == 5) {
          //Luego de los primeros 5 segundo si soltar-> marca que Ya TENGO IUUNB MINIMO de 5 seg. y si suelto ahora
          //ya anulare el nombre de la RED ACTUAL!
          //Pero si sigo pulsado voy en busqueda del borrado general de la EEPTROM DEL MODULO (default)
          YaContol5SegRstSSID = 1;
          digitalWrite(PinLedVerde,HIGH);     //Deja fijo el led
          //CACA
        //DEBERIA ENCENDER UN LED FIJO POR 3 SEGUNDOS PARA DETERMINAR SI CONTINUA LA CUENTA RUMBO AL DEFAULT
        }        

      } else if (CntTimDefault > 9) {      
        Serial.println("Ya conto mas de 10 segundos-> va a general DEFAULT!");
        YaContoTiempoDefault = 1;  //YA gewnera DEFAULT!
        YaContol5SegRstSSID = 0;        //Ya no borra SSID de la red!
        //DebeContarTiempoDefault=0;
        DebeOscErrStatus=1;
      }
    }
    //}

    //________________________
    if (CuentaSegundosPGM1 != 0)    {
      if (CntTimOut1 != 0)      {
        //Si esta contando segundos
        CntTimOut1 = CntTimOut1 - 1;
        if (CntTimOut1 == 0)        {
          DebeDesactivarSalida1 = 1;
          //Serial.println("Debe desactivar salida 1 por segundos");
          DesactivaPorTiempoPGM1 = 1;
        }
      }
    }

    //___
    if (CuentaSegundosPGM2 != 0)    {
      //Serial.println("ESta autorizado a contare segundos de PGM2");
      //Si hay cuenta autorizada de PGM 2 EN SEGUNDOS
      if (CntTimOut2 != 0)
      {
        //Si esta contando PGM2 en segundos
        CntTimOut2 = CntTimOut2 - 1;
        //stringOne = "Tiempo actual cuenta OUT 2 en seg= ";
        //Serial.println(stringOne + CntTimOut2);
        if (CntTimOut2 == 0) {
          DebeDesactivarSalida2 = 1;
          DesactivaPorTiempoPGM2 = 1;
        }
      }
    }

    //______________
    if (CuentaMinutosPGM1 != 0) {
      if (CntSegundosPGM1 == 60) {
        //Serial.println("Conto 1 minuto de PGM 1");
        CntSegundosPGM1 = 0;
        if (CntTimOut1 != 0)        {
          // si esta contando aun tiempo pgm1 en minutos...
          CntTimOut1 = CntTimOut1 - 1;
          //stringOne = "Tiempo actual cuenta minutos OUT 1= ";
          //Serial.println(stringOne + CntTimOut1);
          if (CntTimOut1 == 0)          {
            //Serial.println("Fin tiempo de activacion out 1");
            //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
            DebeDesactivarSalida1 = 1;
            DesactivaPorTiempoPGM1 = 1;
            //Serial.println("Debe desactivar salida 1");
          }
        }
      }                  // no llego a 60 segundos de PGM2 para completar el minuto
      CntSegundosPGM1++; //=CntSegundosPGM1+1;
      //Serial.println(CntSegundosPGM1);
    }

    //____
    if (CuentaMinutosPGM2 != 0) {
      //Si hay cuenta autorizada de PGM 2 en minutos
      if (CntSegundosPGM2 == 60)
      {
        CntSegundosPGM2 = 0;
        if (CntTimOut2 != 0)
        {
          CntTimOut2 = CntTimOut2 - 1;
          //stringOne = "Tiempo actual cuenta minutos OUT 2= ";
          //Serial.println(stringOne + CntTimOut2);
          if (CntTimOut2 == 0)
          {
            //Serial.println("Fin tiempo de activacion out 2");
            //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
            DebeDesactivarSalida2 = 1;
            DesactivaPorTiempoPGM2 = 1;
            //Serial.println("Debe desactivar salida 2 por minutos");
          }
        }
      }
      CntSegundosPGM2++; //=CntSegundosPGM2+1;
      //Serial.println(CntSegundosPGM2);
    }

    //_______________
    if (CuentaHorasPGM1 != 0)    {
      //Si hay cuenta autorizada de PGM 1 en horas
      //Serial.println("Esta cointando tiempo PGM1 en HORAS");
      if (CntMinutosPGM1 == 60)      {
        CntMinutosPGM1 = 0;
        //Serial.println("Conto 1 hora de PGM 1");
        //si esta contando horas en PGM1->
        CntTimOut1 = CntTimOut1 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
        //stringOne = "Tiempo actual cuenta horas OUT 1= ";
        //Serial.println(stringOne + CntTimOut1);
        if (CntTimOut1 == 0)
        {
          //Serial.println("Fin tiempo de activacion out 1 en horas");
          //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
          DebeDesactivarSalida1 = 1;
          DesactivaPorTiempoPGM1 = 1;
          //Serial.println("Debe desactivar salida 1 POR HORAS");
        }
        //conto 1 hora
      }
      //Serial.println("Contador de segundos en modo PGM1 HORAS:");
      CntSegundosPGM1++; //=CntSegundosPGM1+1;
      //Serial.println(CntSegundosPGM1);
      if (CntSegundosPGM1 == 60)
      {
        CntSegundosPGM1 = 0;
        CntMinutosPGM1++; //=CntMinutosPGM1+1;    //Incrementa contadOr minUtos para PGM 1
        //Serial.println("Incrementa contador de minutos de PGM1 en modo HORAS");
        //Serial.println(CntMinutosPGM1);
      }
    }
  }
  //____
  if (CuentaHorasPGM2 != 0)
  {
    //Si hay cuenta autorizada de PGM 1 en horas
    //Serial.println("Esta cointando tiempo PGM1 en HORAS");
    if (CntMinutosPGM2 == 60)
    {
      CntMinutosPGM2 = 0;
      //Serial.println("Conto 1 hora de PGM 1");
      //si esta contando horas en PGM1->
      CntTimOut2 = CntTimOut2 - 1; //DECREMENBTA CUENTA DE APAGADO DE PGM1 EN HORAS
      //stringOne = "Tiempo actual cuenta horas OUT 2= ";
      //Serial.println(stringOne + CntTimOut2);
      if (CntTimOut2 == 0)
      {
        //Serial.println("Fin tiempo de activacion out 2en horas");
        //debe avisar el corte si esta habilitado a hacerlo mediante el ruteo y el telefono de destino
        DebeDesactivarSalida2 = 1;
        DesactivaPorTiempoPGM2 = 1;
        //Serial.println("Debe desactivar salida 2 POR HORAS");
      }
      //conto 1 hora
    }
    //Serial.println("Contador de segundos en modo PGM1 HORAS:");
    CntSegundosPGM2++; //=CntSegundosPGM2+1;
    //Serial.println(CntSegundosPGM2);
    if (CntSegundosPGM2 == 60)
    {
      CntSegundosPGM2 = 0;
      CntMinutosPGM2++; //=CntMinutosPGM2+1;    //Incrementa contadOr minUtos para PGM 1
      //Serial.println("Incrementa contador de minutos de PGM2 en modo HORAS");
      //Serial.println(CntMinutosPGM2);
    }
  }
}

//______________
void GrabaE2_IDReporteProgramado() {
  //resultado = recibido.substring(pos + 5, pos + 15);
  Serial.println("Telefono a guardar : " + resultado);
  int j = 0;
  for (int i = address; i < address + resultado.length(); i++)  {
    EEPROM.write(i, resultado[j]); //Write one by one with starting address of 0x0F
    j = j + 1;
  }
  EEPROM.commit(); //Store data to EEPROM
}

void GrabaE2_UbicacionUsuarios() {
  // resultado = recibido.substring(pos + 16, pos + 36);
  Serial.println("Identificacion del usuario : " + resultado);
  int j = 0;
  for (int i = address; i < address + resultado.length(); i++) {
    EEPROM.write(i, resultado[j]); //Write one by one with starting address of 0x0F
    j = j + 1;
  }
  EEPROM.commit(); //Store data to EEPROM
}

void RdE2() {
  byte value;
  value = EEPROM.read(address);
  Respuesta = value;
}

void WrE2() {
  //EEPROM Write
  EEPROM.write(address, DatoE2);
  EEPROM.commit();
}


void BorraE2() {
  //Serial.println("Borrando E2prom");
  String resultado;
  EEPROM.begin(512);
  String stringOne = "Borrando posicion= ";
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < 1500; i++)
  {
    EEPROM.write(i, 0);
    resultado = stringOne + i;
    Serial.println(resultado);
    delay(20);
  }
  EEPROM.commit();
  //Serial.println("Fin de BORRADO");
}

void GrabaPaqueteE2() {
  int j = 0;
  for (int i = address; i < address + string_variable.length(); i++) {
    EEPROM.write(i, string_variable[j]); //Write one by one with starting address of 0x0F
    j++;                                 //=j+1;
  }
  EEPROM.commit(); //Store data to EEPROM
}

void Carga_Default_ID_Usuario1() {
  address = E2_IDReporte1;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario1();
}

void Carga_Default_ID_Usuario2() {
  address = E2_IDReporte2;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario2();
}

void Carga_Default_ID_Usuario3() {
  address = E2_IDReporte3;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario3();
}

//______
void Carga_Default_ID_Usuario4() {
  address = E2_IDReporte4;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario4();
}

void Carga_Default_ID_Usuario5() {
  address = E2_IDReporte5;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario5();
  //Serial.println("Default ID US5");
}

void Carga_Default_ID_Usuario6() {
  address = E2_IDReporte6;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario6();
}

void Carga_Default_ID_Usuario7() {
  address = E2_IDReporte7;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario7();
}

void Carga_Default_ID_Usuario8() {
  address = E2_IDReporte8;
  string_variable = "?         ";
  GrabaPaqueteE2();
  LeeIDUsuario8();
}

void LeeUbicacionModulo() {
  // lee datos de ubicacion
  address = E2_UbicacionMod;
  ValUbicacionMod = "";
  for (int i = address; i < address + 20; i++)
  {
    ValUbicacionMod = ValUbicacionMod + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  Serial.println("Ubicacion del modulo= " + ValUbicacionMod);
}

void LeeValorUbicacionModulo() {
  address = E2_UbicacionMod;
  ValUbicacionMod = "";
  for (int i = address; i < address + 20; i++)  {
    ValUbicacionMod = ValUbicacionMod + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  //GRAVE ERROR
  //analiza si tiene una coma dentro del texto
  ValorBuscado = ",";
  pos = ValUbicacionMod.indexOf(ValorBuscado);
  if (pos >= 0) {
    Serial.println("ERROR DE AGREGAR UNA COMMA DONDE NO CORRESPONDE");
    ValUbicacionMod.replace(',', ' ');
  }
  Serial.println("Ub.modulo = " + ValUbicacionMod);
}


void LeeDatosMemoria() {
  Serial.println("Leyendo datos de usuarios!");
  LeeIDUsuario1();
  LeeIDUsuario2();
  LeeIDUsuario3();
  LeeIDUsuario4();
  LeeIDUsuario5();
  LeeIDUsuario6();
  LeeIDUsuario7();
  LeeIDUsuario8();

  Serial.println("Valor IdUsuario 1  actual : " + ValIdReporte1);
  Serial.println("Valor IdUsuario 2  actual : " + ValIdReporte2);
  Serial.println("Valor IdUsuario 3  actual : " + ValIdReporte3);
  Serial.println("Valor IdUsuario 4  actual : " + ValIdReporte4);
  Serial.println("Valor IdUsuario 5  actual : " + ValIdReporte5);
  Serial.println("Valor IdUsuario 6  actual : " + ValIdReporte6);
  Serial.println("Valor IdUsuario 7  actual : " + ValIdReporte7);
  Serial.println("Valor IdUsuario 8  actual : " + ValIdReporte8);

  //__________________
  LeeUbicaUsuario1();
  LeeUbicaUsuario2();
  LeeUbicaUsuario3();
  LeeUbicaUsuario4();
  LeeUbicaUsuario5();
  LeeUbicaUsuario6();
  LeeUbicaUsuario7();
  LeeUbicaUsuario8();

  LeeValorClaveAdmin();
  LeeValorClaveUsuario();
  LeeValorUbicacionModulo();
  LeeModoSalidaRemoto();

  LeeValoresRuteo();
  LeeModoUsoPGM1();
  LeeModoUsoPGM2();
  LeeTiempoPGM1();
  LeeTiempoPGM2();
  LeeModoConteoPGM1();
  LeeModoConteoPGM2();
  LeeTiempoPulsoAD();

  LeeTimer1();
  LeeTimer2();
  LeeTimer3();
  LeeTimer4();
  LeeTimer5();
  LeeTimer6();
  LeeTimer7();
  LeeTimer8();
  //LeeClaveAdmin();
  Serial.println("__________________________");
}

//____________
void CreaNuevoUsuario() {
  Serial.println("Detecto Nuevo o actualizacion de ID modulo!");
  Serial.println(variable);
  resultado = variable.substring(pos + 7, pos + 13);
  Serial.println(resultado);
  // es el numero de modulo que es propio y no se guarda

  //________________
  resultado = variable.substring(pos + 14, pos + 18);
  Serial.println(resultado);
  // es la clave y se guarda en eeprom
  address = E2_Clave_Admin;
  string_variable = resultado;
  GrabaPaqueteE2();

  //____________
  resultado = variable.substring(pos + 19, pos + 23);
  Serial.println(resultado);
  address = E2_Clave_Admin;
  ValClaveAdmin = "";
  for (int i = address; i < address + 4; i++)
  {
    ValClaveAdmin = ValClaveAdmin + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  Serial.println("Clave actual = " + ValClaveAdmin);

  //_____
  resultado = variable.substring(pos + 24, pos + 44);
  resultado.replace(',', ' ');
  resultado.replace('"', ' ');
  resultado.replace('}', ' ');
  Serial.println(resultado);
  // Serial.println("Nueva Ubicacion del modulo = ");
  string_variable = resultado;
  address = E2_UbicacionMod;
  GrabaPaqueteE2();
  LeeUbicacionModulo();

  //____
  Serial.println(variable);
  Serial.println("Direccion del usuario que esta solicitando su acceso");
  Serial.println(IdDestino);
  resultado = IdDestino;
  address = posicionWr;
  int j = 0;
  for (int i = address; i < address + resultado.length(); i++)
  {
    EEPROM.write(i, resultado[j]);
    j = j + 1;
  }
  EEPROM.commit();
}

void CargaDefaultUbicacion() {
  address = E2_UbicacionMod;
  string_variable = "ubicacion de modulo ";
  GrabaPaqueteE2();
}

//_______________________________________________________________________
void ChequeaEnvioPaquete() {
  if (HayInternet == 1)  {
    if (DebeResponderAOrigen == 1) {
      DebeResponderAOrigen = 0;
      EnviaComandoADestino();
      DatoRecibido = "";
      //___________
    } else if (DebeDesactivarSalida1 == 1) {
      DebeDesactivarSalida1 = 0;
      CortaSalidaPGM1();
      //___________
    } else if (DebeDesactivarSalida2 == 1) {
      DebeDesactivarSalida2 = 0;
      CortaSalidaPGM2();

      //___________
    } else if (DebeEnviarNotificacionesSegunRuteo == 1) {
      DebeEnviarNotificacionesSegunRuteo = 0;
      Selecciona_RespuestaComun_Notificacion();
      DebeEnviarNotificacionesSegunRuteo = 0;
    }
  } else {
    // Serial.println ("No envio comando por falta de internet");
  }
}

//____________
void LeeValorClaveAdmin() {
  address = E2_Clave_Admin;
  ValClaveAdmin = "";
  for (int i = address; i < address + 4; i++)  {
    ValClaveAdmin = ValClaveAdmin + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  //ValClaveAdmin= ValClaveAdmin(0, 4);
  Serial.println("Clave actual Admin = " + ValClaveAdmin);
}
//______
void LeeValorClaveUsuario() {
  address = E2_Clave_Usua;
  ValClaveUsua = "";
  for (int i = address; i < address + 4; i++)  {
    ValClaveUsua = ValClaveUsua + char(EEPROM.read(i)); //Read one by one with starting address of 0x0F
  }
  Serial.println("Clave actual Usuario= " + ValClaveUsua);
}

//_____________
void LeeValoresRuteo() {
  LeeRuteoID1();
  LeeRuteoID2();
  LeeRuteoID3();
  LeeRuteoID4();
  LeeRuteoID5();
  LeeRuteoID6();
  LeeRuteoID7();
  LeeRuteoID8();
}

void LeeRuteoID1() {
  value = EEPROM.read(E2_Ruteo_ID1);
  stringOne = "Ruteo Id1 = ";
  ValRutId1 = value;
  Serial.println(stringOne + ValRutId1);
}

void LeeRuteoID2() {
  value = EEPROM.read(E2_Ruteo_ID2);
  stringOne = "Ruteo Id2 = ";
  ValRutId2 = value;
  Serial.println(stringOne + ValRutId2);
}

void LeeRuteoID3() {
  value = EEPROM.read(E2_Ruteo_ID3);
  stringOne = "Ruteo Id3 = ";
  ValRutId3 = value;
  Serial.println(stringOne + ValRutId3);
}
void LeeRuteoID4() {
  value = EEPROM.read(E2_Ruteo_ID4);
  stringOne = "Ruteo Id4 = ";
  ValRutId4 = value;
  Serial.println(stringOne + ValRutId4);
}
//__________________
void LeeRuteoID5() {
  address = E2_Ruteo_ID5;
  value = EEPROM.read(address);
  stringOne = "Ruteo Id5 = ";
  ValRutId5 = value;
  Serial.println(stringOne + ValRutId5);
}
//__________________
void LeeRuteoID6() {
  address = E2_Ruteo_ID6;
  value = EEPROM.read(address);
  stringOne = "Ruteo Id6 = ";
  ValRutId6 = value;
  Serial.println(stringOne + ValRutId6);
}
//__________________
void LeeRuteoID7() {
  address = E2_Ruteo_ID7;
  value = EEPROM.read(address);
  stringOne = "Ruteo Id7 = ";
  ValRutId7 = value;
  Serial.println(stringOne + ValRutId7);
}
//__________________
void LeeRuteoID8() {
  address = E2_Ruteo_ID8;
  value = EEPROM.read(address);
  stringOne = "Ruteo Id8 = ";
  ValRutId8 = value;
  Serial.println(stringOne + ValRutId8);
}

//______
void LeeModoUsoPGM1() {
  address = E2ModoPGM1;
  ValModoPGM1 = EEPROM.read(address);
  //ValModoPGM1 = value;
  Serial.println("Modo PGM1= " + String(ValModoPGM1));
}
//______
void LeeModoUsoPGM2() {
  address = E2ModoPGM2;
  ValModoPGM2 = EEPROM.read(address);
  // = value;
  Serial.println("Modo PGM2= " + String(ValModoPGM2));
}
//_________________
void LeeModoConteoPGM1() {
  address = E2_TIPO_TIM1;
  ValModoCuentaPGM1 = EEPROM.read(address);
  Serial.println("Modo HMS 1= " + String(ValModoCuentaPGM1));
}
//____________
void LeeModoConteoPGM2() {
  address = E2_TIPO_TIM2;
  ValModoCuentaPGM2 = EEPROM.read(address);
  Serial.println("Modo HMS 2= " + String(ValModoCuentaPGM2));
}
//__________
void LeeTiempoPGM1() {
  address = E2TimPGM1;
  ValTimPGM1 = EEPROM.read(address);
  Serial.println("Tiempo PGM1= " + String(ValTimPGM1));
}
//______________
void LeeTiempoPGM2() {
  address = E2TimPGM2;
  ValTimPGM2 = EEPROM.read(address);
  Serial.println("Tiempo PGM2= " + String(ValTimPGM2));
}
//_________
//tOTAL DE TIEMPO DE PULSO
void LeeTiempoPulsoAD() {
  address = E2TimPulAD;
  ValTimOutAD = EEPROM.read(address);
  Serial.println("Tiempo pulso A / D = " + String(ValTimOutAD));
}

//__________________________________________________
void LeeModoSalidaRemoto() {
  address = E2SalidaAD;
  ValModoOutAD = EEPROM.read(address);
  //01=Modo nivel
  //02=modo pulso de determinada duracion
  if (ValModoOutAD == 2) {
    resultado = "Es modo PULSO";
  } else {
    resultado = "Es modo NIVEL";
  }
  Serial.println("Modo Remoto = " + resultado);
}

//_______________________________________
void AnalizaStringRecibido() {
  if (DatoRecibido != "")  {
    DatoRecibido.replace('** DONATIONWARE **', ' ');
    Serial.println("Se va a procesar este mensaje -> " + DatoRecibido);

    variable = DatoRecibido;
    DatoRecibido = "";
    for (int i = 0; i < 2; i++)  {
      digitalWrite(PinLedAmarillo, HIGH);      //LOW);
      if (HayInternet == 1) {
        delay(100);
      } else {
        delay(400);
      }
      digitalWrite(PinLedAmarillo, LOW);      //HIGH);
      if (HayInternet == 1) {
        delay(100);
      } else {
        delay(400);
      }
    }

    //1- Determina datos a procesar y quien lo envió!(IDOrigenEnvio)
    //String ValorBuscado;
    ValorBuscado = "&to=";
    pos = variable.indexOf(ValorBuscado);
    //Serial.println(" busqueda de id origen");
    if (pos >= 0) {

      IdModuloDestinatario = variable.substring(pos + 4, pos + 10);
      //Serial.println("Modulo buscado ; " + IdModuloDestinatario);
    }
    //Serial.println("Busqueda de valor 'data'...");
    ValorBuscado = "data";
    DebeOscErrStatus = 0;
    pos = variable.indexOf(ValorBuscado);
    if (pos >= 0) {

      recibido = variable.substring(pos + 5, pos + 17);
      //Serial.println(recibido);
      //Serial.println("___");

      ValorBuscado = "from";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        IdDestino = variable.substring(pos + 5, pos + 15);
        UsuarioEmisor = IdDestino;
        //Serial.println("Enviado desde : " + String(IdDestino));                //Recupera valor IMEI del emisor del comando
        //Serial.println("Moulo destinatario : " + OrigenTopicoEnvio);
        Serial.println("________________");
      }
    }

    //_________________________________________________________
    if (IdModuloDestinatario == NumSerialESP || EsModoShare == 1)  {
      EsModoShare = 0;
      ValorBuscado = "solicitadatos";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        // Analiza si el numero de id del modulo es el que corresponde al modulo
        ValUbicacionMod.trim();
        Serial.println("Ubicacion de modulo = " + ValUbicacionMod);

        if (ValUbicacionMod == "ubicacion de modulo") {

          //Caso MODULO NUEVO
          BuscaCoincideIdUsua();
          if (ExisteUsuario == 1) {
            Serial.println("TELEFONO EXISTENTE- MODULO NUEVO ");
            //caso TELEFONO NUEVO - MODULO NUEVO
            //Si no tiene datos de ubicacion de modulo y tampoco el numero de usuario recibido, le envia sus datos como default al usuario
          } else {
            Serial.println("TELEFONO NUEVO- MODULO NUEVO ");
            //caso TELEFONO EXISTENTE - MODULO NUEVO
            //Si no tiene datos de ubicacion pero si dispone del ID del usuaio,va a recibir del usuario la actualizacion de datoa desde el telefojo (UPLOAD1).
          }

        } else {
          //Caso MODULO EXISTENTE"
          BuscaCoincideIdUsua();
          if (ExisteUsuario == 1) {
            //Serial.println("TELEFONO EXISTENTE- MODULO EXISTENTE ");
            //caso TELEFONO EXISTENTE - MODULO EXISTENTE
            //si ya tiene datos DE UBICACION y del usuario, le envia los datos propios como actualizacion de modulo a usuario
          } else {
            Serial.println("TELEFONO NUEVO  MODULO EXISTENTE ");
            //TELEFONO NUEVO - MODULO EXISTENTE
            //Si tiene datos de ubicacion pero no coincide el ID de usuaio, le envia a este sus datos para que se actualice el usuario desde los datos de telefdono.
          }
        }
        EnviaDatosModuloAUsuario();
      }
      //____________________
      ValorBuscado = "SHARE";
      //Serial.println("Va a leer si hay share");
      //SHARE ORIGEN ADM,ID MOD
      //no se envian datos de otros usuario, solo los que debe aprender este modulo para ingresar al sistema
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        Serial.println(pos);
        Serial.println(variable);
        Serial.println("Procesa datos de COMPARTIR");
        temporal = variable.substring(pos + 6, pos + 12);
        //if (temporal == NumSerialESP) {

        ImeiAutorizado = variable.substring(pos + 13, pos + 23);  //Destino push el imei que va a agregarse por share
        Serial.println("Imei Usuario compartido : " + ImeiAutorizado);
        IdOrigen = ImeiAutorizado;

        //_____________
        String UbicacionAutorizada = variable.substring(pos + 24, pos + 44);  //Ubicacion del modulo asignada
        Serial.println("Ubicacion asignada : " + UbicacionAutorizada);

        //_____________
        String IdentificaAutorizado = variable.substring(pos + 45, pos + 65);  //Nombre o ID asignada al usuario
        Serial.println("Nombre del nuevo usuario : " + IdentificaAutorizado);
        GeneradorPublicacion = ImeiAutorizado;
        Serial.println(GeneradorPublicacion);
        Serial.println(IdDestino);

        //_____________
        resultado = variable.substring(pos + 66, pos + 67);    //Posicion de la memoria eeprom donde guardar los datos de Usuario e ID
        posicion = resultado.toInt();
        Serial.println("Posicion de memoria donde guarda la info : ");
        Serial.println(posicion);

        if (posicion == 1) {
          //Crea datos para usuario 1
          Serial.println("Hay prog.DE ID REPORTE 1!");

          resultado = ImeiAutorizado;
          address = E2_IDReporte1;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion

          //______
          BorraE2promNombreUsuario1();
          resultado = IdentificaAutorizado;
          address = E2_UbicacionID1;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario1();
          LeeUbicaUsuario1();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          CmdAccion = "accion-OKtel 1," + NumSerialESP + "," + ValIdReporte1 + "," + ValUbicacionID1 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);

          DebeResponderAOrigen = 1;

          //_________________________________________________
        } else if (posicion == 2) {
          Serial.println("Hay prog.DE ID REPORTE 2!");

          resultado = ImeiAutorizado;
          address = E2_IDReporte2;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario2();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID2;
          GrabaE2_UbicacionUsuarios();

          LeeIDUsuario2();
          LeeUbicaUsuario2();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 2," + NumSerialESP + "," + ValIdReporte2 + "," + ValUbicacionID2 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //____________
        } else if (posicion == 3) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte3;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario3();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID3;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario3();
          LeeUbicaUsuario3();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 3," + NumSerialESP + "," + ValIdReporte3 + "," + ValUbicacionID3 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //_________________
        } else if (posicion == 4) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte4;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario4();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID4;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario4();
          LeeUbicaUsuario4();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 4," + NumSerialESP + "," + ValIdReporte4 + "," + ValUbicacionID4 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //_____________
        } else if (posicion == 5) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte5;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario5();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID5;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario5();
          LeeUbicaUsuario5();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 5," + NumSerialESP + "," + ValIdReporte5 + "," + ValUbicacionID5 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //_____________________
        } else if (posicion == 6) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte6;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario6();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID6;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario6();
          LeeUbicaUsuario6();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 6," + NumSerialESP + "," + ValIdReporte6 + "," + ValUbicacionID6 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //___________________
        } else if (posicion == 7) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte7;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario7();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID7;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario7();
          LeeUbicaUsuario7();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 7," + NumSerialESP + "," + ValIdReporte7 + "," + ValUbicacionID7 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;

          //___________________
        } else if (posicion == 8) {
          resultado = ImeiAutorizado;
          address = E2_IDReporte8;
          GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
          BorraE2promNombreUsuario8();

          resultado = IdentificaAutorizado;
          address = E2_UbicacionID8;
          GrabaE2_UbicacionUsuarios();
          LeeIDUsuario8();
          LeeUbicaUsuario8();

          string_variable = UbicacionAutorizada;
          address = E2_UbicacionMod;
          GrabaPaqueteE2();
          LeeUbicacionModulo();

          GeneradorPublicacion = ImeiAutorizado;
          CmdAccion = "accion-OKtel 8," + NumSerialESP + "," + ValIdReporte8 + "," + ValUbicacionID8 + "," + UbicacionAutorizada;
          Serial.println(CmdAccion);
          DebeResponderAOrigen = 1;
        }
      }

      //_________________
      ValorBuscado = "cambiaubica";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        Serial.println(variable);
        address = E2_UbicacionMod;
        string_variable = "                    ";
        GrabaPaqueteE2();

        resultado = variable.substring(pos + 12, pos + 18);
        Serial.println("Modulo buscado : " + resultado);

        resultado = variable.substring(pos + 19, pos + 39);
        Serial.println("Nueva ubicacion : " + resultado);


        string_variable = resultado;
        address = E2_UbicacionMod;
        GrabaPaqueteE2();
        LeeUbicacionModulo();
        CmdAccion = "cambioubica OK";
        Serial.println(CmdAccion);
        DebeResponderAOrigen = 1;

      }
      //_________________
      //Analiza texto TEMPOR y el horario/fecha de temporizaciones
      ValorBuscado = "tempor";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        Serial.println(string_variable);
        temporal = variable.substring(pos + 7, pos + 8);
        string_variable = variable.substring(pos + 9, pos + 21);
        tempo = temporal.toInt();
        switch (tempo) {
          case 0:
            Serial.println("_____________");
            Serial.println("Guarda en E2_TIMER1");
            address = E2_TIMER1;
            GrabaPaqueteE2();
            LeeTimer1();
            break;

          case 1:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER2");
            address = E2_TIMER2;
            GrabaPaqueteE2();
            LeeTimer2();
            break;

          case 2:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER3");
            address = E2_TIMER3;
            GrabaPaqueteE2();
            LeeTimer3();
            break;

          case 3:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER4");
            Serial.println(string_variable);
            address = E2_TIMER4;
            GrabaPaqueteE2();
            LeeTimer4();
            break;

          case 4:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER5");
            address = E2_TIMER5;
            GrabaPaqueteE2();
            LeeTimer5();
            break;

          case 5:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER6");
            address = E2_TIMER6;
            GrabaPaqueteE2();
            LeeTimer6();
            break;

          case 6:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER7");
            address = E2_TIMER7;
            GrabaPaqueteE2();
            LeeTimer7();
            break;

          case 7:
            Serial.println("_____________");
            Serial.println("Guarda en  E2_TIMER8");
            address = E2_TIMER8;
            GrabaPaqueteE2();
            LeeTimer8();
            break;

          default:
            //si nada coincide, hace lo predeterminado
            // default es optional
            break;
        }
      }

      ValorBuscado = "upload1";
      int pos1 = variable.indexOf(ValorBuscado);
      if (pos1 >= 0)  {
        Serial.println(pos1);
        //Serial.println("BUFFER A ANALIZAR : " + String(variable));

        //Para el caso que le llegue desde un servidor todos los datos junto s actualizacion!!
        Serial.println("________________________________________");
        int TotalRecibido = variable.length();

        String BufferLimpio;
        BufferLimpio = variable.substring(pos1, pos1 + TotalRecibido);
        Serial.println(BufferLimpio);
        Serial.println("__");
        pos1 = 8;

        string_variable = BufferLimpio.substring(pos1, pos1 + 10);
        Serial.println("ID usua 1 : " +  string_variable);
        address = E2_IDReporte1;
        GrabaPaqueteE2();
        LeeIDUsuario1();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 11, pos1 + 21);
        //string_resultado.trim();
        Serial.println("ID usua 2 : " +  string_variable);
        address = E2_IDReporte2;
        GrabaPaqueteE2();
        LeeIDUsuario2();

        //__________
        string_variable = BufferLimpio.substring(pos1 + 22, pos1 + 32);
        Serial.println("ID usua 3 : " +  string_variable);
        address = E2_IDReporte3;
        GrabaPaqueteE2();
        LeeIDUsuario3();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 33, pos1 + 43);
        //string_resultado.trim();
        Serial.println("ID usua 4 : " +  string_variable);
        address = E2_IDReporte4;
        GrabaPaqueteE2();
        LeeIDUsuario4();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 44, pos1 + 54);
        //string_variable.trim();
        Serial.println("ID usua 5 : " +  string_variable);
        address = E2_IDReporte5;
        GrabaPaqueteE2();
        LeeIDUsuario5();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 55, pos1 + 65);
        //string_variable.trim();
        Serial.println("ID usua 6 : " +  string_variable);
        address = E2_IDReporte6;
        GrabaPaqueteE2();
        LeeIDUsuario6();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 66, pos1  + 76);
        //string_variable.trim();
        //Serial.println("ID usua 7 : " +  string_variable);
        address = E2_IDReporte7;
        GrabaPaqueteE2();
        LeeIDUsuario7();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 77, pos1 + 87);
        //Serial.println("ID usua 8 : " +  string_variable);
        address = E2_IDReporte8;
        GrabaPaqueteE2();
        LeeIDUsuario8();

        //__________________________________________________________________________________________________
        string_variable = BufferLimpio.substring(pos1 + 88, pos1 + 108);
        //Serial.println("ubicacion usua 1 : " +  string_variable);
        BorraE2promNombreUsuario1();
        address = E2_UbicacionID1;
        GrabaPaqueteE2();
        LeeUbicaUsuario1();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 109, pos1 + 129);
        //Serial.println("ubicacion usua 2 : " +  string_variable);
        BorraE2promNombreUsuario2();
        address = E2_UbicacionID2;
        GrabaPaqueteE2();
        LeeUbicaUsuario2();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 130, pos1 + 150);
        //Serial.println("ubicacion usua 3 : " +  string_variable);
        BorraE2promNombreUsuario3();
        address = E2_UbicacionID3;
        GrabaPaqueteE2();
        LeeUbicaUsuario3();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 151, pos1 + 171);
        //Serial.println("ubicacion usua 4 : " +  string_variable);
        BorraE2promNombreUsuario4();
        address = E2_UbicacionID4;
        GrabaPaqueteE2();
        LeeUbicaUsuario4();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 172, pos1 + 192);
        //Serial.println("ubicacion usua 5 : " +  string_variable);
        BorraE2promNombreUsuario5();
        address = E2_UbicacionID5;
        GrabaPaqueteE2();
        LeeUbicaUsuario5();
        //___________
        string_variable = BufferLimpio.substring(pos1 + 193, pos1 + 213);
        //Serial.println("ubicacion usua 6 : " +  string_variable);
        BorraE2promNombreUsuario6();
        address = E2_UbicacionID6;
        GrabaPaqueteE2();
        LeeUbicaUsuario6();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 214, pos1 + 234);
        //Serial.println("ubicacion usua 7 : " +  string_variable);
        BorraE2promNombreUsuario7();
        address = E2_UbicacionID7;
        GrabaPaqueteE2();
        LeeUbicaUsuario7();

        //___________
        string_variable = BufferLimpio.substring(pos1 + 235, pos1 + 255);
        //Serial.println("ubicacion usua 8 : " +  string_variable);
        BorraE2promNombreUsuario8();
        address = E2_UbicacionID8;
        GrabaPaqueteE2();
        LeeUbicaUsuario8();
        CmdAccion = "OK UPLOAD1";
        Serial.println("Acepto datos del telefono que reprograma el modulo parte 1 ");
        DebeResponderAOrigen = 1;
      }

      //______________________________________________________________________________
      ValorBuscado = "upload2";
      pos = variable.indexOf(ValorBuscado);
      if (pos >= 0) {
        //Taller Francia,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,3,3,2,3,2,1


        Serial.println("Recibio EL SEGUNDO PAQUETE DE DATOS PARA ACTUALIZAR EL MODULO CON DATOS EN EL TELEFONO");
        Serial.println(variable);
        //___________________________
        string_variable = variable.substring(pos + 8, pos + 28);

        Serial.println("Ubicacion modulo: " + string_variable);
        address = E2_UbicacionMod;
        GrabaPaqueteE2();
        LeeUbicacionModulo();



        //________________________________________________
        DatoE2 = 0;
        //Serial.println("ruteo 1");
        resultado = variable.substring(pos + 29, pos + 30);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 1;
        }
        //________
        resultado = variable.substring(pos + 31, pos + 32);
        int posicion  = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 2;
        }

        //___
        resultado = variable.substring(pos + 33, pos + 34);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 4;
        }
        //___
        resultado = variable.substring(pos + 35, pos + 36);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 8;
        }
        Serial.println(" Ruteo ID 1 : " +  String(DatoE2));
        address = E2_Ruteo_ID1;     //guarda la union de todos los bytes
        WrE2();

        //_____________________________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 37, pos + 38);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 39, pos + 40);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 41, pos + 42);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 43, pos + 44);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 8;
        }

        Serial.println("Ruteo ID 2 : " +  String(DatoE2));
        address = E2_Ruteo_ID2;
        WrE2();

        //___________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 45, pos + 46);
        posicion = resultado.toInt();

        if (posicion == 1) {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 47, pos + 48);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 49, pos + 50);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 51, pos + 52);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1) {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID3");
        Serial.println("Ruteo ID 3 :" +  String(DatoE2));
        address = E2_Ruteo_ID3;
        WrE2();

        //______________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 53, pos + 54);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 55, pos + 56);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 57, pos + 58);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 59, pos + 60);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID4");
        Serial.println("Ruteo ID 4 :" +  String(DatoE2));
        address = E2_Ruteo_ID4;
        WrE2();

        //__________________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 61, pos + 62);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 63, pos + 64);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 65, pos + 66);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 67, pos + 68);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID5");
        Serial.println("Ruteo ID 5 :" +  String(DatoE2));
        address = E2_Ruteo_ID5;
        WrE2();


        //_________________________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 69, pos + 70);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 71, pos + 72);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 73, pos + 74);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 75, pos + 76);
        //Serial.println(posicion);
        posicion = resultado.toInt();
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID6");
        Serial.println("Ruteo ID 6 :" +  String(DatoE2));
        address = E2_Ruteo_ID6;
        WrE2();
        //_______________________________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 77, pos + 78);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 79, pos + 80);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 81, pos + 82);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 83, pos + 84);
        //Serial.println(posicion);
        posicion = resultado.toInt();
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID7");
        Serial.println("Ruteo ID 7 :" +  String(DatoE2));
        address = E2_Ruteo_ID7;
        WrE2();


        //__________________________________________________
        DatoE2 = 0;
        resultado = variable.substring(pos + 85, pos + 86);
        //Serial.println(posicion);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 1;
        }
        //____
        resultado = variable.substring(pos + 87, pos + 88);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 2;
        }
        //____
        resultado = variable.substring(pos + 89, pos + 90);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 4;
        }
        //____
        resultado = variable.substring(pos + 91, pos + 92);
        posicion = resultado.toInt();
        //Serial.println(posicion);
        if (posicion == 1)
        {
          DatoE2 = DatoE2 | 8;
        }
        //Serial.println("resultado de Ruteo ID8");
        Serial.println("Ruteo ID 8 :" +  String(DatoE2));
        address = E2_Ruteo_ID8;
        WrE2();

        //____________________________________________
        resultado = variable.substring(pos + 93, pos + 94);
        Serial.println(resultado);
        address = E2SalidaAD;
        DatoE2 = resultado.toInt();
        WrE2();                       // Modo de generar  armado/desarmado!!

        Serial.print("Modo Salida AD: ");
        Serial.println(DatoE2);

        //_____
        resultado = variable.substring(pos + 95, pos + 96);
        Serial.println(resultado);
        address = E2ModoPGM1;
        DatoE2 = resultado.toInt();
        Serial.println("Modo PGM 1:" +  String(DatoE2));
        WrE2();

        //_____
        resultado = variable.substring(pos + 97, pos + 98);
        Serial.println(resultado);
        address = E2ModoPGM2;
        DatoE2 = resultado.toInt();
        Serial.println("Modo PGM 2:" +  String(DatoE2));
        WrE2();

        //_____   -
        resultado = variable.substring(pos + 99, pos + 100);   //(pos + 105, pos + 106);             //OJO  LOS TIEMPOs EN 2 BYTES
        Serial.println(resultado);
        address = E2_TIPO_TIM1;
        DatoE2 = resultado.toInt();
        Serial.println("Modo Cuenta TIM 1:" +  String(DatoE2));
        WrE2();

        //_____
        resultado = variable.substring(pos + 101, pos + 102);   //(pos + 107, pos + 108);             //OJO  LOS TIOEMPO EN 2 BYTES
        Serial.println(resultado);
        address = E2_TIPO_TIM2;
        DatoE2 = resultado.toInt();
        Serial.println("TIPO TIM 2:" +  String(DatoE2));
        WrE2();

        //_____________
        resultado = variable.substring(pos + 103, pos + 104);   //(pos + 99, pos + 100);
        Serial.println(resultado);
        address = E2TimPulAD;
        DatoE2 = resultado.toInt();
        Serial.println("Tim PULSO AD:" +  String(DatoE2));
        WrE2();

        //______________________________________________________
        resultado = variable.substring(pos + 105, pos + 107);
        Serial.println(resultado);
        address = E2TimPGM1;
        DatoE2 = resultado.toInt();
        Serial.println("Tiempo de PGM1 : " + String(DatoE2));
        WrE2();

        //_____
        resultado = variable.substring(pos + 108, pos + 110);
        Serial.println(resultado);
        address = E2TimPGM2;
        DatoE2 = resultado.toInt();
        Serial.println("Tiempo de PGM 2:" +  String(DatoE2));
        WrE2();

        LeeDatosMemoria();

        //________
        CmdAccion = "OK UPLOAD2";
        DebeResponderAOrigen = 1;
      }

      //_____________________________________
      pos = variable.indexOf("newmod");
      if (pos >= 0) {
        Serial.println(pos);  //31 EL OFFSET EN 31!!

        //from=736f66746d&to=0430ec&data=newmod 0430ec,oficina 430ec,Alejandro           ,736f66746d,1

        //Serial.println("Valor autorizado de telefono administrador");
        //Serial.println(ImeiAutorizado);       //no recibio ningun dato

        //Serial.println("_____________________");
        //Serial.println(variable.substring(pos + 13, pos + 14));       //44-45deberia haber un ,
        //Serial.println(variable.substring(pos + 34, pos + 35));     //deberia haber un ,
        //Serial.println(variable.substring(pos + 55, pos + 56));
        //Serial.println(variable.substring(pos + 67, pos + 68));

        if (variable.substring(pos + 34, pos + 35) != ",") {
          Serial.println("Esta desfasado la recepcion y no debe guardar!");
          return;
        }

        Serial.println("_____________________");

        resultado = variable.substring(pos + 7, pos + 13);            //Id de modulo
        Serial.println("Id modulo : " + resultado);

        string_variable = variable.substring(pos + 14, pos + 34);     // texto ubicacion modulo
        Serial.println("Ubicacion modulo : " + string_variable);
        TxtUbicaModulo = string_variable;

        string_variable = variable.substring(pos + 35, pos + 55);     //Texto NOMBRE usuario a crear
        Serial.println("Nombre usuario : " + string_variable);
        NombreUsuarioNuevo = string_variable;

        //___________________-
        string_variable = variable.substring(pos + 56, pos + 66);   //  IMEI de nuevo usuario
        IdImeiUsuarioNuevo = string_variable;
        Serial.println("El imei que sera su publicador= " + IdImeiUsuarioNuevo);

        //______________
        string_variable = variable.substring(pos + 67, pos + 68);     //ubicacion donde guardar
        DatoE2 = string_variable.toInt();
        Serial.println("_______________________________");


        //ahora guarda en meorioa segunposicion que le hay dicho
        //cambio 21/4/20
        //si el imei ya estaba en la memoria-> fue creado por otro por lo tanto no cambia los datos aqui!

        byte ImeiExistente = 0;

        if (IdImeiUsuarioNuevo == ValIdReporte1 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte2 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte3 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte4 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte5 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte6 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte7 ) {
          ImeiExistente = 1;

        } else if (IdImeiUsuarioNuevo == ValIdReporte8 ) {
          ImeiExistente = 1;
        }

        //if (ImeiExistente == 0) {
        //  Serial.println("No existe ese usuario previamente, puede guardar sus datos");
        address = E2_UbicacionMod;
        string_variable = TxtUbicaModulo;
        GrabaPaqueteE2();
        LeeUbicacionModulo();

        //determina posicion libre para asignar nuevo usuario
        if (DatoE2 == 1) {
          BorraE2promNombreUsuario1();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID1;
          GrabaPaqueteE2();

          LeeUbicaUsuario1();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte1;
          GrabaPaqueteE2();
          LeeIDUsuario1();
          Serial.println("Valor IdUsuario 1 : " + ValIdReporte1);
          Serial.println("___________");

        } else if (DatoE2 == 2) {
          BorraE2promNombreUsuario2();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID2;
          GrabaPaqueteE2();
          LeeUbicaUsuario2();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte2;
          GrabaPaqueteE2();
          LeeIDUsuario2();
          Serial.println("Valor IdUsuario 2 : " + ValIdReporte2);
          Serial.println("___________");

        } else if (DatoE2 == 3) {
          BorraE2promNombreUsuario3();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID3;
          GrabaPaqueteE2();
          LeeUbicaUsuario3();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte3;
          GrabaPaqueteE2();
          LeeIDUsuario3();
          Serial.println("Valor IdUsuario 3 : " + ValIdReporte3);
          Serial.println("___________");

        } else if (DatoE2 == 4) {
          BorraE2promNombreUsuario4();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID4;
          GrabaPaqueteE2();
          LeeUbicaUsuario4();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte4;
          GrabaPaqueteE2();
          LeeIDUsuario4();
          Serial.println("Valor IdUsuario 4 : " + ValIdReporte4);
          Serial.println("___________");

        } else if (DatoE2 == 5) {
          BorraE2promNombreUsuario5();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID5;
          GrabaPaqueteE2();
          LeeUbicaUsuario5();

          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte5;
          GrabaPaqueteE2();
          LeeIDUsuario5();
          Serial.println("Valor IdUsuario 5 : " + ValIdReporte5);
          Serial.println("___________");

        } else if (DatoE2 == 6) {
          BorraE2promNombreUsuario6();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID6;
          GrabaPaqueteE2();
          LeeUbicaUsuario6();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte6;
          GrabaPaqueteE2();
          LeeIDUsuario6();
          Serial.println("Valor IdUsuario 6 : " + ValIdReporte6);
          Serial.println("___________");

        } else if (DatoE2 == 7) {
          BorraE2promNombreUsuario7();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID7;
          GrabaPaqueteE2();
          LeeUbicaUsuario7();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte7;
          GrabaPaqueteE2();
          LeeIDUsuario7();
          Serial.println("Valor IdUsuario 7 : " + ValIdReporte7);
          Serial.println("___________");

        } else if (DatoE2 == 8) {
          BorraE2promNombreUsuario8();
          string_variable = NombreUsuarioNuevo;
          address = E2_UbicacionID8;
          GrabaPaqueteE2();
          LeeUbicaUsuario2();
          string_variable = IdImeiUsuarioNuevo;
          address = E2_IDReporte8;
          GrabaPaqueteE2();
          LeeIDUsuario8();
          Serial.println("Valor IdUsuario 8 : " + ValIdReporte8);
          Serial.println("___________");
        }

        EnviaDatosModuloAUsuario();


        //CmdAccion = "newmod " + NumSerialESP + "," + ValUbicacionMod + "," + NombreUsuarioNuevo + "," + IdImeiUsuarioNuevo + "," + String(DatoE2);
        variable = "";
        DatoRecibido = "";
      }

      //________________________________
      pos = variable.indexOf("modpgm1");
      if (pos >= 0) {
        Serial.println("Detecto comando MODPGM1!");
        // procesar comando modo de uso pgm1
        Serial.println(variable);
        Serial.println("__________________________");
        resultado = variable.substring(pos + 8, pos + 9);
        DatoE2 = resultado.toInt();
        address = E2ModoPGM1;
        WrE2();
        LeeModoUsoPGM1();

        //_________________________________________
        pos = variable.indexOf("modpgm2");
        //Serial.println ("Detecto comando MODPGM2!");
        resultado = variable.substring(pos + 8, pos + 9);
        DatoE2 = resultado.toInt();
        address = E2ModoPGM2;
        WrE2();
        LeeModoUsoPGM2();

        //________________
        pos = variable.indexOf("timpgm1");
        resultado = variable.substring(pos + 8, pos + 9); //pos8 pos10
        //Serial.println("Modo de conteo 1");
        DatoE2 = resultado.toInt();
        address = E2_TIPO_TIM1;
        WrE2();
        LeeModoConteoPGM1();

        resultado = variable.substring(pos + 10, pos + 12);
        DatoE2 = resultado.toInt();
        address = E2TimPGM1;
        WrE2();
        LeeTiempoPGM1();

        //_______________________________________________
        pos = variable.indexOf("timpgm2");
        resultado = variable.substring(pos + 8, pos + 9); //pos8 pos10
        DatoE2 = resultado.toInt();
        address = E2_TIPO_TIM2;
        WrE2();
        LeeModoConteoPGM2();
        resultado = variable.substring(pos + 10, pos + 12);
        DatoE2 = resultado.toInt();
        address = E2TimPGM2;
        WrE2();
        LeeTiempoPGM2();

        //_________________________________
        // DECODIFICA datos DE MODO DE USO SALIDA remota PARA armar/desarmar equipo externo
        pos = variable.indexOf("outremoto");
        resultado = variable.substring(pos + 10, pos + 11);
        Serial.println("Modo salida AD = " + String(resultado));
        DatoE2 = resultado.toInt();
        address = E2SalidaAD; //tiempo del pulso de armado/desarmado
        WrE2();
        //2=pulso?
        LeeModoSalidaRemoto();
        Serial.println("Modo Remoto = " + String(ValModoOutAD));

        //__________
        //Ubica texto timpulso
        pos = variable.indexOf("timpulso");
        resultado = variable.substring(pos + 9, pos + 11);
        //Serial.println ("Tim.pulso Remoto");
        resultado.replace(', ', ' ');
        resultado.replace('"', ' ');
        resultado.replace('}', ' ');
        //Serial.println(resultado);
        DatoE2 = resultado.toInt();
        address = E2TimPulAD; //tiempo del pulso de armado/desarmado
        WrE2();

        LeeTiempoPulsoAD();
        ComandoActualizaValores = "1234 ";
        ComandoActualizaValores = ComandoActualizaValores + "modpgm1 " + ValModoPGM1;
        ComandoActualizaValores = ComandoActualizaValores + ", modpgm2 " + ValModoPGM2;
        ComandoActualizaValores = ComandoActualizaValores + ", timpgm1 " + ValModoCuentaPGM1 + ", " + ValTimPGM1;
        ComandoActualizaValores = ComandoActualizaValores + ", timpgm2 " + ValModoCuentaPGM2 + ", " + ValTimPGM2;
        ComandoActualizaValores = ComandoActualizaValores + ", outremoto " + ValModoOutAD + ", timpulso " + ValTimOutAD;
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;

      }

      //___________________
      pos = variable.indexOf("borrausuario");
      if (pos >= 0) {
        Serial.println ("Detecto BORRAR USUARIO!");
        Serial.println(variable);
        GeneradorPublicacion = IDOrigenEnvio;
        resultado = variable.substring(pos + 13, pos + 14);
        posicion = resultado.toInt();
        if (posicion == 1) {
          CmdAccion = "borrausua 1";
          EnviaComandoADestino();
          Serial.println("Va a borra los datos de usuario y nombre 1");
          Carga_Default_ID_Usuario1();     //Limpia todos los datos del usuario 1 en memoria
          BorraE2promNombreUsuario1();

        } else if (posicion == 2) {
          CmdAccion = "borrausua 2";
          EnviaComandoADestino();
          Serial.println("Va a borra los datos de usuario y nombre 2");
          Carga_Default_ID_Usuario2();
          BorraE2promNombreUsuario2();
        } else if (posicion == 3) {
          //Serial.println("debe enviar comando BORRA USUARIO 3");
          CmdAccion = "borrausua 3";
          EnviaComandoADestino();
          Serial.println("Va a borra los datos de usuario y nombre 3");
          Carga_Default_ID_Usuario3();
          BorraE2promNombreUsuario3();

        } else if (posicion == 4) {
          //Serial.println("debe enviar comando BORRA USUARIO 4");
          CmdAccion = "borrausua 4";
          Serial.println("Va a borra los datos de usuario y nombre 4");
          EnviaComandoADestino();
          Carga_Default_ID_Usuario4();
          BorraE2promNombreUsuario4();

        } else if (posicion == 5) {
          //Serial.println("debe enviar comando BORRA USUARIO 5");
          CmdAccion = "borrausua 5";
          EnviaComandoADestino();
          Carga_Default_ID_Usuario5();
          BorraE2promNombreUsuario5();

        } else if (posicion == 6) {
          //Serial.println("debe enviar comando BORRA USUARIO 6");
          CmdAccion = "borrausua 6";
          EnviaComandoADestino();
          Carga_Default_ID_Usuario6();
          BorraE2promNombreUsuario6();

        } else if (posicion == 7) {
          //Serial.println("debe enviar comando BORRA USUARIO 7");
          CmdAccion = "borrausua 7";
          EnviaComandoADestino();
          Carga_Default_ID_Usuario7();
          BorraE2promNombreUsuario7();

        } else if (posicion == 8) {
          //Serial.println("debe enviar comando BORRA USUARIO 8");
          CmdAccion = "borrausua 8";
          EnviaComandoADestino();
          Carga_Default_ID_Usuario8();
          BorraE2promNombreUsuario8();
        } else {
          Serial.println("NO ENTIENDIO EL NUMERO!!");
        }
        LeeDatosMemoria();
      }

      //___________________
      pos = variable.indexOf("borramod");
      if (pos >= 0)
      {
        Serial.println("Detecto BORRAR PERFIL!");
        Serial.println(variable);
        resultado = variable.substring(pos + 9, pos + 15);
        resultado.replace(',', ' ');
        resultado.replace('"', ' ');
        resultado.replace('}', ' ');
        Serial.println(resultado);
        CargaDefault();
        CmdAccion = "borramod ";
        CmdAccion = CmdAccion + resultado;
        Serial.println(CmdAccion);
        DebeResponderAOrigen = 1;
        LeeDatosMemoria();
      }

      //__________
      pos = variable.indexOf("activar");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);

        Serial.println("Detecto comando ACTIVAR!");
        DebeOscErrStatus = 0; //DEJA DE OSCILAR LES STATUS

        CmdAccion = "Sistema armado";
        Descripcion = CmdAccion;
        DebeEnviarNotificacionesSegunRuteo = 1;

        LeeModoSalidaRemoto();        //caca determina que tipo de accion hece para ARMADO O DESARMADO!
        if (ValModoOutAD == 1) {
          //si es modo NIVEL
          digitalWrite(PinOutRemoto, LOW); //PONE SALIDA A MASA para ACTIVAR!
          Serial.println("Es modo NIVEL y pasa a LOW cuando es ACTIVAR");

        } else {
          LeeTiempoPulsoAD();
          Serial.println("Esta en MODO ACTIVAR pero genera modo PULSO de HIGH TO LOW");
          digitalWrite(PinOutRemoto, HIGH); //ON led de salida ARM/DESARM
          delay(ValTimOutAD * 1000);
          digitalWrite(PinOutRemoto, LOW);
          DatoE2 = 1;                     //esta activado
          address = E2_ST_ARMED;
          WrE2();               //Es ARMADO
        }
        //CmdAccion = "Sistema armado";
        //DebeEnviarNotificacionesSegunRuteo = 1;
      }

      //________________
      pos = variable.indexOf("desarmar");
      if (pos >= 0) {
        Serial.println("Detecto comando DESARMAR!");
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        DebeOscErrStatus = 0; //DEJA DE OSCILAR LES STATUS
        CmdAccion = "Sistema desarmado";
        DebeEnviarNotificacionesSegunRuteo = 1;

        LeeModoSalidaRemoto();           //caca determina que tipo de accion hece para ARMADO O DESARMADO!
        if (ValModoOutAD == 1) {
          digitalWrite(PinOutRemoto, HIGH); //PONE SALIDA A "1" para DESACTIVAR!
          Serial.println("Es modo NIVEL y pasa a HIGH cuando es DESACTIVA");

        } else {
          Serial.println("Es modo PULSO y genera de HIGH TO LOW");
          LeeTiempoPulsoAD();
          digitalWrite(PinOutRemoto, HIGH); //ON led de salida ARM/DESARM
          delay(ValTimOutAD * 1000);
          digitalWrite(PinOutRemoto, LOW);
          DatoE2 = 0;
          address = E2_ST_ARMED;
          WrE2();               //Es DESARMADO
        }
      }

      //_____________________________________
      pos = variable.indexOf("panrem");
      if (pos >= 0) {
        Serial.println("Detecto PANREM");
        //procesar datos de activar PANICO REMOTO
        CmdAccion = "PANICO TELEFONICO";
        IDOrigenEnvio = "dispositivo";
        Selecciona_RespuestaComun_Notificacion();
      }

      //_______________________________________________________________________________________________
      pos = variable.indexOf("on_out1");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        Serial.println("Recibido : " + variable);

        Serial.println("ACTIVA PGM1");
        digitalWrite(PinOut1Pgm, HIGH);
        StPGM1 = 1;
        LeeModoConteoPGM1();
        Serial.println("Modo HMS 1= " + String(ValModoCuentaPGM1));
        LeeModoUsoPGM1();
        LeeTiempoPGM1();

        if (ValModoPGM1 == 2 || ValModoPGM1 == 4) {
          CntTimOut1 = ValTimPGM1;        //contador decremental
          if (ValModoCuentaPGM1 == 1) {
            Serial.println("Cuenta tiempo PGM 1 EN SEGUNDOS");
            CuentaSegundosPGM1 = 1; //habilita cuenta en SEGUNDOS para PGM 1

          } else if (ValModoCuentaPGM1 == 2) {
            Serial.println("Cuenta tiempo PGM1 en MINUTOS");
            CuentaMinutosPGM1 = 1; // habilita cuenta en MINUTOS para PGM 1
            CntSegundosPGM1 = 0;

          } else if (ValModoCuentaPGM1 == 3) {
            CuentaHorasPGM1 = 1; // habilita cuenta en HORAS para PGM 1
            CntSegundosPGM1 = 0;
            CntMinutosPGM1 = 0;
            Serial.println("Cuenta tiempo PGM1 en HORAS");
          }
        }
        CmdAccion = "Activacion salida 1";
        Serial.println("Nombre usuario emisor : " + String(NombreUsuaEmisor));
        DebeEnviarNotificacionesSegunRuteo = 1;
        ArmaEstadoInOutActual();
        Serial.println(resultado);
      }

      //________
      pos = variable.indexOf("off_out1");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        CortaSalidaPGM1();
      }
      //________
      pos = variable.indexOf("on_out2");
      if (pos >= 0)  {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);

        Serial.println("ACTIVA PGM2");
        digitalWrite(PinOut2Pgm, HIGH);
        StPGM2 = 1;
        DebeEnviarNotificacionesSegunRuteo = 1;
        LeeTiempoPGM2();
        CntTimOut2 = ValTimPGM2;
        //EsEventoPGM2 = 1;
        //Serial.println("Conecta SALIDA PGM 2");
        LeeModoConteoPGM2();
        LeeModoUsoPGM2();
        Serial.println("Tiempo de activacion PGM2 :");
        Serial.println(ValTimPGM2);
        DesactivaPorTiempoPGM2 = 0;
        if (ValModoPGM2 == 2 || ValModoPGM2 == 4)
        {
          Serial.println("ACTIVA PGM2 Y EMPIEZA CUENTA)");
          CntTimOut2 = ValTimPGM2;
          if (ValModoCuentaPGM2 == 1) {
            Serial.println("Cuenta tiempo PGM2 en SEGUNDOS");
            CuentaSegundosPGM2 = 1; /// habilita cuenbta en SEGUNDOS para PGM 2
            CntSegundosPGM1 = 0;
          } else if (ValModoCuentaPGM2 == 2) {
            Serial.println("Cuenta tiempo PGM2 en MINUTOS");
            CuentaMinutosPGM2 = 1; // habilita cuenbta en MINUTOS para PGM 2
            CntSegundosPGM2 = 0;
            CntMinutosPGM2 = 0;
          } else {
            Serial.println("Cuenta tiempo PGM2 en HORAS");
            CuentaHorasPGM2 = 1; // habilita cuenbta en HORAS para PGM 2
            CntHorasPGM2 = 0;
          }
        }
        CmdAccion = "Activacion salida 2";
        ArmaEstadoInOutActual();
        Serial.println(resultado);
        Serial.println(NombreUsuaEmisor);
        DebeEnviarNotificacionesSegunRuteo = 1;
        //Serial.println("________________");
      }

      //________
      pos = variable.indexOf("off_out2");
      if (pos >= 0) {
        pos = variable.indexOf(",");
        NombreUsuaEmisor = variable.substring(pos + 1, pos + 21);
        CortaSalidaPGM2();
      }

      //______________
      pos = variable.indexOf("ruteo 1 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 1!");
        address = E2_Ruteo_ID1;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID1(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 1 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 2 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 2!");
        address = E2_Ruteo_ID2;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID2(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 2 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 3 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 3!");
        address = E2_Ruteo_ID3;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID3(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 3 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 4 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 4!");
        resultado = variable.substring(pos + 8, pos + 11);
        address = E2_Ruteo_ID4;
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID4(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 4 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 5 ");
      if (pos >= 0)    {
        Serial.println("Hay prog.RUTEO 5!");
        address = E2_Ruteo_ID5;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID5(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 5 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 6 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 6!");
        address = E2_Ruteo_ID6;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID6(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 6 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 7 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 7!");
        address = E2_Ruteo_ID7;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID7(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 7 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("ruteo 8 ");
      if (pos >= 0) {
        Serial.println("Hay prog.RUTEO 8!");
        address = E2_Ruteo_ID8;
        resultado = variable.substring(pos + 8, pos + 11);
        DatoE2 = resultado.toInt();
        WrE2();
        LeeRuteoID8(); //Lee valor de ruteo telefono ID
        CmdAccion = "cambio ruteo 8 ";
        CmdAccion = CmdAccion + DatoE2;
        DebeResponderAOrigen = 1;
      }

      //_______________________________________________________________-_
      pos = variable.indexOf("TEL1 ");
      if (pos >= 0)  {
        //para el caso que reciba un texto de actualizacion de  usuario 1 + su identificacion
        Serial.println(variable);
        Serial.println("Hay prog.DE ID REPORTE 1!");
        //recibido = variable;

        address = E2_IDReporte1;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado(); // decodifica q10 posiciones del usuario y 20 de la ubicacion
        BorraE2promNombreUsuario1();
        address = E2_UbicacionID1;

        //_____________
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario1();
        LeeUbicaUsuario1();
        CmdAccion = "accion-OKtel 1," + IdOrigen + "," + ValIdReporte1 + "," + ValUbicacionID1 + ",";
        //Serial.println(ComandoActualizaValores);
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("TEL2 ");
      if (pos >= 0) {
        Serial.println("Hay prog.DE ID REPORTE 2!");
        address = E2_IDReporte2;
        recibido = variable;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();
        BorraE2promNombreUsuario2();
        address = E2_UbicacionID2;
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario2();
        LeeUbicaUsuario2();
        ComandoActualizaValores = "accion-OKtel 2," + IdOrigen + "," + ValIdReporte2 + "," + ValUbicacionID2 + ",";
        //Serial.println(ComandoActualizaValores);
        //Serial.println("Actualiza a todos los usuario los datos del usuario 2");
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("TEL3 ");
      if (pos >= 0) {
        Serial.println("Hay prog.DE ID REPORTE 3!");
        recibido = variable;
        address = E2_IDReporte3;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();

        BorraE2promNombreUsuario3();
        address = E2_UbicacionID3;
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario3();
        LeeUbicaUsuario3();
        ComandoActualizaValores = "accion-OKtel 3, " + IdOrigen + "," + ValIdReporte3 + "," + ValUbicacionID3 + ",";
        //Serial.println(ComandoActualizaValores);
        //Serial.println("Actualiza a todos los usuario los datos del usuario 3");
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("TEL4 ");
      if (pos >= 0) {
        Serial.print(variable);
        Serial.println("Hay prog.DE ID REPORTE 4!");
        address = E2_IDReporte4;
        recibido = variable;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();
        BorraE2promNombreUsuario4();
        address = E2_UbicacionID4;
        //debe bortrar previamente los datos del usuario de la eeprom
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario4();
        LeeUbicaUsuario4();
        ComandoActualizaValores = "accion-OKtel 4," + IdOrigen + "," + ValIdReporte4 + "," + ValUbicacionID4 + ",";
        Serial.println(ComandoActualizaValores);
        //Serial.println("Actualiza a todos los usuario los datos del usuario 4");
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;

      }
      //______________
      pos = variable.indexOf("TEL5 ");
      if (pos >= 0) {
        address = E2_IDReporte5;
        recibido = variable;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();

        BorraE2promNombreUsuario5();
        address = E2_UbicacionID5;
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        Serial.println("Hay prog.de ID REPORTE 5!");
        LeeIDUsuario5();
        LeeUbicaUsuario5();
        ComandoActualizaValores = "accion-OKtel 5," + IdOrigen + "," + ValIdReporte5 + "," + ValUbicacionID5 + ",";
        Serial.println(ComandoActualizaValores);
        CmdAccion = ComandoActualizaValores;
      }

      //______________
      pos = variable.indexOf("TEL6 ");
      if (pos >= 0) {
        Serial.println("Hay prog.DE ID REPORTE 6!");
        recibido = variable;
        address = E2_IDReporte6;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();

        BorraE2promNombreUsuario6();
        address = E2_UbicacionID6;
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario6();
        LeeUbicaUsuario6();
        ComandoActualizaValores = "accion-OKtel 6," + IdOrigen + "," + ValIdReporte6 + "," + ValUbicacionID6 + ",";
        Serial.println(ComandoActualizaValores);
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;
      }
      //______________
      pos = variable.indexOf("TEL7 ");
      if (pos >= 0) {
        Serial.println("Hay prog.DE ID REPORTE 7!");
        recibido = variable;
        address = E2_IDReporte7;
        resultado = recibido.substring(pos + 5, pos + 15);
        GrabaE2_IDReporteProgramado();

        BorraE2promNombreUsuario7();
        address = E2_UbicacionID7;
        resultado = recibido.substring(pos + 16, pos + 36);

        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario7();
        LeeUbicaUsuario7();
        ComandoActualizaValores = "accion-OKtel 7," + IdOrigen + "," + ValIdReporte7 + "," + ValUbicacionID7 + ",";
        Serial.println(ComandoActualizaValores);
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;
      }

      //______________
      pos = variable.indexOf("TEL8 ");
      if (pos >= 0) {
        Serial.println("Hay prog.DE ID REPORTE 8!");
        recibido = variable;
        address = E2_IDReporte8;
        resultado = recibido.substring(pos + 5, pos + 15);
        BorraE2promNombreUsuario8();
        address = E2_UbicacionID8;
        resultado = recibido.substring(pos + 16, pos + 36);
        GrabaE2_UbicacionUsuarios();
        LeeIDUsuario8();
        LeeUbicaUsuario8();
        ComandoActualizaValores = "accion-OKtel 8," + IdOrigen + "," + ValIdReporte8 + "," + ValUbicacionID8 + ",";
        Serial.println(ComandoActualizaValores);
        CmdAccion = ComandoActualizaValores;
        DebeResponderAOrigen = 1;
      }

      //________
      pos = variable.indexOf("estado");
      if (pos >= 0) {
        Serial.println("ESTADO ACTUAL");
        ArmaEstadoInOutActual();
        Serial.println(resultado);

        CmdAccion = resultado;
        DebeResponderAOrigen = 1;
        Serial.println(CmdAccion);
      }
    }
  }
}


void LeeClaveAdmin() {
  address = E2_CLV_ADMIN;
  RamValClvAdmin = LeeComunIDUsuario();
  Serial.println("Clave administrado telefonos1 : " + RamValClvAdmin);
}

void LeeIDUsuario1() {
  address = E2_IDReporte1;
  ValIdReporte1 = LeeComunIDUsuario();
  //Serial.println("Valor IdUsuario 1  actual : " + );
  pos = ValIdReporte1.indexOf("?");
  if (pos >= 0)  {
    HayUsuario1 = 0;
  } else  {
    HayUsuario1 = 1;
  }
}

void LeeIDUsuario2() {
  address = E2_IDReporte2;
  ValIdReporte2 = LeeComunIDUsuario();
  pos = ValIdReporte2.indexOf("?");
  if (pos >= 0)  {
    HayUsuario2 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario2 = 1;
  }
}


void LeeIDUsuario3() {
  address = E2_IDReporte3;
  ValIdReporte3 = LeeComunIDUsuario();
  pos = ValIdReporte3.indexOf("?");
  if (pos >= 0)  {
    HayUsuario3 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario3 = 1;
  }
}

void LeeIDUsuario4() {
  address = E2_IDReporte4;
  ValIdReporte4 = LeeComunIDUsuario();
  pos = ValIdReporte4.indexOf("?");
  if (pos >= 0)  {
    HayUsuario4 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario4 = 1;
  }
}

void LeeIDUsuario5() {
  address = E2_IDReporte5;
  ValIdReporte5 = LeeComunIDUsuario();
  pos = ValIdReporte5.indexOf("?");
  if (pos >= 0)  {
    HayUsuario5 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario5 = 1;
  }
}

void LeeIDUsuario6() {
  address = E2_IDReporte6;
  ValIdReporte6 = LeeComunIDUsuario();
  pos = ValIdReporte6.indexOf("?");
  if (pos >= 0)  {
    HayUsuario6 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario6 = 1;
  }
}

void LeeIDUsuario7() {
  address = E2_IDReporte7;
  ValIdReporte7 = LeeComunIDUsuario();
  pos = ValIdReporte7.indexOf("?");
  if (pos >= 0)  {
    HayUsuario7 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario7 = 1;
  }
}

void LeeIDUsuario8() {
  address = E2_IDReporte8;
  ValIdReporte8 = LeeComunIDUsuario();
  pos = ValIdReporte8.indexOf("?");
  if (pos >= 0)  {
    HayUsuario8 = 0;
    //Serial.println("Numero inexistente");
  } else  {
    HayUsuario8 = 1;
  }
}

String LeeComunIDUsuario() {
  String ValIdUsuario = "";
  for (int i = address; i < address + 10; i++) {
    ValIdUsuario = ValIdUsuario + char(EEPROM.read(i));
  }
  ValIdUsuario = ValIdUsuario.substring(0, 10);
  return ValIdUsuario;
}

//_________________________________________________________________________________________
void LeeUbicaUsuario1() {
  address = E2_UbicacionID1;
  ValUbicacionID1 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.1 = " + ValUbicacionID1);
}

void LeeUbicaUsuario2() {
  address = E2_UbicacionID2;
  ValUbicacionID2 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.2 = " + ValUbicacionID2);
}

void LeeUbicaUsuario3() {
  address = E2_UbicacionID3;
  ValUbicacionID3 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.3 = " + ValUbicacionID3);
}

void LeeUbicaUsuario4() {
  address = E2_UbicacionID4;
  ValUbicacionID4 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.4 = " + ValUbicacionID4);
}

void LeeUbicaUsuario5() {
  address = E2_UbicacionID5;
  ValUbicacionID5 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.5 = " + ValUbicacionID5);
}

void LeeUbicaUsuario6() {
  address = E2_UbicacionID6;
  ValUbicacionID6 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.6 = " + ValUbicacionID6);
}

void LeeUbicaUsuario7() {
  address = E2_UbicacionID7;
  ValUbicacionID7 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.7 = " + ValUbicacionID7);
}

void LeeUbicaUsuario8() {
  address = E2_UbicacionID8;
  ValUbicacionID8 = LeeComunUbicaUsua();
  Serial.println("Nombre usu.8 = " + ValUbicacionID8);
}

//________________________________
String LeeComunUbicaUsua()
{
  String ValUbicaUsua = "";
  for (int i = address; i < address + 20; i++)
  {
    ValUbicaUsua = ValUbicaUsua + char(EEPROM.read(i));
  }
  ValUbicaUsua = ValUbicaUsua.substring(0, 20);
  return ValUbicaUsua;
}


//_______
void BorraE2promNombreUsuario1() {
  address = E2_UbicacionID1;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario2() {
  address = E2_UbicacionID2;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario3() {
  address = E2_UbicacionID3;
  borraBloqueE2prom();
}
//_______
void BorraE2promNombreUsuario4() {
  address = E2_UbicacionID4;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario5() {
  address = E2_UbicacionID5;
  borraBloqueE2prom();
}
//_______
void BorraE2promNombreUsuario6() {
  address = E2_UbicacionID6;
  borraBloqueE2prom();
}

//_______
void BorraE2promNombreUsuario7() {
  address = E2_UbicacionID7;
  borraBloqueE2prom();
}
//_______
void BorraE2promNombreUsuario8() {
  address = E2_UbicacionID8;
  borraBloqueE2prom();
}

//_____________
void borraBloqueE2prom() {
  resultado1 = string_variable;
  string_variable = "?                   ";
  GrabaPaqueteE2();
  string_variable = resultado1;
}

//________________________________________________________________
//lectura de reloj para agregar al envio de novedad
void digitalClockDisplay() {
  int digits;
  HoraFechaActual = "";
  printDigitsN2p(hour());
  //HoraFechaActual=HoraFechaActual + hour();
  //HoraFechaActual=HoraFechaActual + " : ";

  //___
  printDigits(minute());
  //HoraFechaActual= HoraFechaActual + digits;
  //HoraFechaActual= HoraFechaActual + minute();

  printDigits(second());
  //_________
  //HoraFechaActual=HoraFechaActual + " : ";
  //HoraFechaActual= HoraFechaActual + digits;

  //___________________________
  HoraFechaActual = HoraFechaActual + " - ";
  if (digits < 10)
    HoraFechaActual = HoraFechaActual;
  HoraFechaActual = HoraFechaActual + day();
  printfecha(month());
  printfecha(year());
  //Serial.println("Hora a agregar a la notificacion : ");
  Serial.println(HoraFechaActual);
}

//_____________
void printfecha(int digits)
{
  HoraFechaActual = HoraFechaActual + '/';
  if (digits < 10)
    HoraFechaActual = HoraFechaActual + '0';
  HoraFechaActual = HoraFechaActual + digits;
}

//_____________
void printDigitsN2p(int digits)
{
  if (digits < 10)
    HoraFechaActual = HoraFechaActual + '0';
  HoraFechaActual = HoraFechaActual + digits;
}
//_____________
void printDigits(int digits)
{
  HoraFechaActual = HoraFechaActual + ':';
  if (digits < 10)
    HoraFechaActual = HoraFechaActual + '0';
  HoraFechaActual = HoraFechaActual + digits;
}

//_________________________________________
const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address
  while (Udp.parsePacket() > 0)
    ; // discard any previously received packets
  //Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);

  //Serial.print(ntpServerName);
  //Serial.print(" : ");
  //Serial.println(ntpServerIP);

  //__________________
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500)
  {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      //Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  //Serial.println("No NTP Response : -(");
  return 0; // return 0 if unable to get the time


}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//____________
void LeeInAuxiliar() {
  //return;
  /*
    if (digitalRead(PinInAux) == 0)  {
    if (StatusIn == 0) {
      Serial.println("Entrada aux a masa");
      StatusIn = 1;
    }
    } else {
    if (StatusIn == 1) {
      Serial.println("Entradas AUX a POSITIVO");
      StatusIn = 0;
    }
    }
  */
  if (digitalRead(PinInAux) == 0)  {
    if (HayDeteccionAux == 0) {
      HayDeteccionAux = 1;
      DebeContarTiempoDefault = 1;
      CntTimDefault = 0;        //empieza a contar tiempo para borrar red o default!
      YaContol5SegRstSSID = 0;      //Inicializa flag de resultado final
      YaContoTiempoDefault = 0;
      Serial.println("Debe empezar a contar 1 segundo y alternar led ON / OFF");
    }

  } else {
    // no esta pulsado boton de programacion
    if (HayDeteccionAux == 1) {
      HayDeteccionAux = 0;         //ya no detecta mas
      DebeOscErrStatus=0;       //anula oscilacion por las dudas que se haya activado x tiempo

      if (DebeContarTiempoDefault == 1) {
        Serial.println("Detecta que solto pulsador AUXILIAR!");
        // ANALIZA CUANTOS SEGUNDOS ESTUVO DETECTANDO PARA DETERMINAR QUE ACCION REALIZAR
        DebeContarTiempoDefault = 0;    //Termina con la cuenta de ACCION de pulsador
        if (YaContol5SegRstSSID == 1 && YaContoTiempoDefault==0 ) {
          Serial.println("Ya contando mas de 5 segundos -> ");
          YaContol5SegRstSSID == 0; //ACEPTA QUE CONTO MAS DE 5 SEG PERO MENOS DE 10 SEGUNDOS                    
          //se genera borrado de red por pulsacion constante pero desdpues de dos reset
          Serial.println("Pide nuevos datos de red!");
          DatoE2 = 1;
          address = E2_BORRA_RED;
          WrE2();
          //delay(100);
          WiFiManager wifiManager;
          wifiManager.resetSettings();
          
          digitalWrite(PinLedVerde, LOW);
          CntTimDefault = 0;
          YaContoTiempoDefault = 0;
          YaContol5SegRstSSID = 0;
          TotalLeds = 4;
          GeneraBeepsLed();          
          ESP.reset();       // Reactivar!




        } else if (YaContoTiempoDefault == 1) {
          //aun no conto 10 segundos y siu suelta solo
          YaContoTiempoDefault == 0; //Acepta que conto mas de 6 pulsos
          Serial.println("CARGA DEFAULT!");
          CargaDefault();
          //queda encendido led amarillo hasta que hay algun usuario !!!!!!!!!!!!!!!!!!!!
          CntTimDefault = 0;
          YaContoTiempoDefault = 0;
          YaContol5SegRstSSID = 0;
          TotalLeds = 6;
          GeneraBeepsLed();
        } 
        CntTimDefault = 0;
        //YaContoTiempoDefault = 0;
        YaContol5SegRstSSID = 0;
      }
    }
  }
}

//________________
void ArmaEstadoInOutActual() {
  resultado = "ESTADO ACTUAL ";
  LeeStSirena();
  if (StSir == 0)  {
    resultado = resultado + "SIR_0,";
  } else  {
    resultado = resultado + "SIR1,";
  }
  //________________
  //SUPER CACA
  //Si no esta conectado el cable el cable der retorno de ARTM/DESAR->  por el nivel termina dando
  //DESARMADO!!! Aunque se haya armado ! y depende si es por NIVEL O PULSO!
  //Como el pulso siempre termina en MASA-> LO TOMA COMO DESARMADO!
  //Unica solucion es guardar en memoria eeprom cual fue el ultimno estado!??
  //caca determina que tipo de accion hece para ARMADO O DESARMADO!
  if (ValModoOutAD == 2) {
    //Por ser PULSO -> dependera del ultimo estado
    ArmaEstadoArmInhxPulso();

  } else {
    //Es NIVEL-> puede leer el verdadero estado
    // sin embargo, puede estar mal el cable de sensado, -> entonces verificar si corresponde
    //el nivel con lo que tiene programado!!!
    if (digitalRead(PinArmDesarm) == 0)  {         // Detecta entra a masa (ojo que la SIRENA puede ser una MASA o un ALTO
      delay(100);
      if (digitalRead(PinArmDesarm) == 0) {
        resultado = resultado + "ARMED,";
        Serial.println("Estado -> ARMADO");
      }
    } else {
      resultado = resultado + "DISARM,";
      Serial.println("Estado -> DESARMADO");
    }
  }
  //________
  if (digitalRead(PinOut1Pgm) == 0) {
    delay(100);
    if (digitalRead(PinOut1Pgm) == 0) {
      StPGM1 = 0;
      resultado = resultado + "PGM1_0,";
    }
  } else {
    delay(100);
    if (digitalRead(PinOut1Pgm) == 1) {
      StPGM1 = 1;
      resultado = resultado + "PGM1_1,";
    }
  }

  //_______________
  if (digitalRead(PinOut2Pgm) == 0)  {
    delay(100);
    if (digitalRead(PinOut2Pgm) == 0) {
      StPGM2 = 0;
      resultado = resultado + "PGM2_0,";
    }
    //Serial.println("Leyo salida 2 determinando que esta en 0");

  } else {
    delay(100);
    if (digitalRead(PinOut2Pgm) == 1) {
      StPGM2 = 1;
      //Serial.println("Leyo salida 2 determinando que esta en 1");       //
      resultado = resultado + "PGM2_1,";
    }
  }
}

void  ArmaEstadoArmInhxPulso() {
  address = E2_ST_ARMED;
  RdE2();
  if (Respuesta == 1) {
    EstaArmado = 1;
    resultado = resultado + "ARMED,";

  } else {
    //Serial.println("Ultimo estado = DESARMADO");
    EstaArmado = 0;
    resultado = resultado + "DISARM,";
  }
}

void CortaSalidaPGM1() {
  Serial.println("DESACTIVA PGM1");
  digitalWrite(PinOut1Pgm, LOW);
  CuentaSegundosPGM1 = 0; //ya no cuenta en segundos
  CuentaMinutosPGM1 = 0;
  CuentaHorasPGM1 = 0;
  StPGM1 = 0;

  CntTimOut1 = 0;         //Anula tiempo de pgm 1
  if (DesactivaPorTiempoPGM1 == 1)  {
    //Serial.println("El corte de la salida 1 fue por tiempo");
  } else {
    //Serial.println("El corte de la salida 1 fue por accion de usuario");
  }

  CmdAccion = "Desactivacion salida 1";
  Descripcion = CmdAccion;
  DesactivaPorTiempoPGM1 = 0;
  // envia quien es el telefono que solicito la accion
  ArmaEstadoInOutActual();
  Serial.println(resultado);
  Serial.println(NombreUsuaEmisor);
  DebeEnviarNotificacionesSegunRuteo = 1;
}

//___________________
void CortaSalidaPGM2() {
  digitalWrite(PinOut2Pgm, LOW);
  CuentaSegundosPGM2 = 0; //ya no cuenta en segundos
  CuentaMinutosPGM2 = 0;
  CuentaHorasPGM2 = 0;
  StPGM2 = 0;
  CntTimOut2 = 0;               //Anula tiempo de pgm 2
  CmdAccion = "Desactivacion salida 2";
  Descripcion = CmdAccion;
  Serial.println(NombreUsuaEmisor);
  DebeEnviarNotificacionesSegunRuteo = 1;
  DesactivaPorTiempoPGM2 = 0;
  ArmaEstadoInOutActual();
  //Serial.println(resultado);
}


//____________________________________________________________________________________________
void CargaDefTimer1() {
  string_variable = "00000-------"; //activa PGM 1 todos los dias a la misma hora (incluidos domingos
  address = E2_TIMER1;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer2() {
  string_variable = "00000-------";     //Desactiva PGM 1todos los dias a la misma hora
  address = E2_TIMER2;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer3() {
  string_variable = "00000-------"; //Activa PGM2 solo los martes a la hora preijada
  address = E2_TIMER3;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer4() {
  string_variable = "00000-------"; //Desactiva PGM2 solo los Miercoles a la hora prefijada
  address = E2_TIMER4;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer5() {
  string_variable = "00000-------"; //Genera ARMED solo los jueves a la hora prefijada
  address = E2_TIMER5;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer6() {
  string_variable = "00000-------"; //Genera DISARMED solo los Viernes a la hora prefijada
  address = E2_TIMER6;
  GrabaPaqueteE2();
}
//_____
void CargaDefTimer7() {
  string_variable = "00000-------"; //ON PGM3 solo dias Sabado a la hora prefijada
  address = E2_TIMER7;
  GrabaPaqueteE2();
}
//L M M J V S D
//_____
void CargaDefTimer8() {
  string_variable = "00000-------";    //"17574LMNJVS_"; //Apaga PGM3 todos los dias a la hora prefijada
  address = E2_TIMER8;
  GrabaPaqueteE2();
}

void ArmaFiltroDiaSemana() {
  d = RamDiaSemanaActual.toInt(); //numero del dia de la semana actual
  //Serial.println("Dia semana actual : " + String(d));
  CoincideDiaSemanaActual = 0;
  //_________
  switch (d) {
    case 1:
      //Si hoy es Domingo-> debe enmascarar 1 y RamSelDias para bloque timer analizado
      valCompara = "D";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el DOMINGO como dia de analisis!");
      } else {
        //Serial.println("No tiene domingo como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 2:
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el LUNES como dia de analisis!");
      } else {
        //Serial.println("No tiene LUNES como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 3:
      valCompara = "M";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el MARTES como dia de analisis!");
      } else {
        //Serial.println("No tiene MARTES como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 4:
      valCompara = "N";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el MIERCOLES como dia de analisis!");
      } else {
        //Serial.println("No tiene MIERCOLES como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 5:
      valCompara = "J";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        Serial.println("Tiene el JUEVES como dia de analisis!");
      } else {
        //Serial.println("No tiene JUEVES como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 6:
      valCompara = "V";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el VIERNES como dia de analisis!");
      } else {
        //Serial.println("No tiene VIERNES como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    case 7:
      valCompara = "S";
      pos = resultado.indexOf(valCompara);
      if (pos >= 0) {
        CoincideDiaSemanaActual = 1;
        //Serial.println("Tiene el SABADO como dia de analisis!");
      } else {
        //Serial.println("No tiene SABADO como dia elegido");
        CoincideDiaSemanaActual = 0;
      }
      break;

    default:
      Serial.println("NO ES DIA VALIDO ");
      //si nada coincide, hace lo predeterminado
      break;
  }
}

void DefineTipoAccionxTim() {
  switch (tempo) {
    case 1:
      Serial.println("ACTIVA PGM1");
      digitalWrite(PinOut1Pgm, HIGH);
      break;

    case 2:
      digitalWrite(PinOut1Pgm, LOW);
      Serial.println("Desactiva PGM1");
      break;

    case 3:
      Serial.println("ACTIVA PGM2");
      digitalWrite(PinOut2Pgm, HIGH);
      break;

    case 4:
      Serial.println("Desactiva PGM2");
      digitalWrite(PinOut1Pgm, LOW);
      break;

    case 5:
      Serial.println("GENERA PULSO PARA ARMAR");
      if (ValModoOutAD == 1) {
        digitalWrite(PinOutRemoto, LOW); //Off led de salida ARM/DESARM
      } else {
        LeeTiempoPulsoAD();
        digitalWrite(PinOutRemoto, HIGH); //ON led de salida ARM/DESARM
        delay(ValTimOutAD * 1000);
        digitalWrite(PinOutRemoto, LOW);
      }
      break;

    case 6:
      Serial.println("GENERA PULSO PARA DESARMAR");
      if (ValModoOutAD == 1) {
        digitalWrite(PinOutRemoto, LOW); //Off led de salida ARM/DESARM
      } else {
        LeeTiempoPulsoAD();
        digitalWrite(PinOutRemoto, HIGH); //ON led de salida ARM/DESARM
        delay(ValTimOutAD * 1000);
        digitalWrite(PinOutRemoto, LOW);
      }
      break;
    /*
        case 7:
          Serial.println("ACTIVA PGM3");
          digitalWrite(PinOut1Pgm, HIGH);
          break;

        case 8:
          digitalWrite(PinOut1Pgm, LOW);
          Serial.println("Desactiva PGM3");
          break;

        case 9:
          Serial.println("ACTIVA PGM4");
          digitalWrite(PinOut2Pgm, HIGH);
          break;
        case 0:
          Serial.println("Desactiva PGM4");
          digitalWrite(PinOut1Pgm, LOW);
          break;
    */
    default:
      //si nada coincide, hace lo predeterminado
      // default es optional
      break;
  }
}

void GetExternalIP() {
  WiFiClient client1;
  if (!client1.connect("api.ipify.org", 80)) {

    if (HayAvisoFaltaInternet == 0) {
      Serial.println("Falla conexion internet externa - sin ip publica");
      cntFaltaInternet++;
      reconnect();
      if (cntFaltaInternet > 3) {
        Serial.println("Confirmado falta de internet");
        cntFaltaInternet = 0;
        DebeOscErrStatus = 1; //debe oscilar por falta de internet
        HayInternet = 0;
        HayAvisoFaltaInternet = 1;
      }
    } else {
      //Ya detecto antes falta de internet
    }

  } else {
    if (HayAvisoFaltaInternet == 1) {
      Serial.print("Hubo aviso falta de internet");
    }

    int timeout = millis() + 5000;
    client1.print("GET / ? format = json HTTP / 1.1\r\nHost : api.ipify.org\r\n\r\n");
    while (client1.available() == 0) {
      if (timeout - millis() < 0) {
        //Serial.println(" >>> Client Timeout !");
        client1.stop();
        return;
      }
    }
    int size;
    while ((size = client1.available()) > 0) {
      uint8_t *msg = (uint8_t *)malloc(size);
      size = client1.read(msg, size);
      free(msg);
      if (HayInternet == 0) {
        HayInternet = 1;
        DebeOscErrStatus = 0; //debe oscilar por falta de internet
      }
      cntFaltaInternet = 0;
      DebeOscErrStatus = 0;
      if (HayAvisoFaltaInternet == 1) {
        HayAvisoFaltaInternet = 01;
        digitalWrite(PinLedAmarillo, LOW);
        Serial.println("Hay IP publica");
      }
    }
  }
}

void EnviaDatosModuloAUsuario() {
  //LeeDatosMemoria();
  ValIdReporte1.trim();
  ValIdReporte2.trim();
  ValIdReporte3.trim();
  ValIdReporte4.trim();
  ValIdReporte5.trim();
  ValIdReporte6.trim();
  ValIdReporte7.trim();
  ValIdReporte8.trim();

  ValUbicacionID2.trim();
  ValUbicacionID3.trim();
  ValUbicacionID4.trim();
  ValUbicacionID5.trim();
  ValUbicacionID6.trim();
  ValUbicacionID7.trim();
  ValUbicacionID8.trim();
  Serial.println(IdDestino);

  ArmaEstadoInOutActual();
  Serial.println(resultado);

  String CmdAccion = "datosmodulo " + NumSerialESP + "," + ValClaveAdmin + "," + ValClaveUsua + "," + ValUbicacionMod + "," + ValIdReporte1 + "," + ValIdReporte2 + ","
                     + ValIdReporte3 + "," + ValIdReporte4 + "," + ValIdReporte5 + "," + ValIdReporte6 + "," + ValIdReporte7 + "," + ValIdReporte8 + ","
                     + ValUbicacionID1 + "," + ValUbicacionID2 + "," + ValUbicacionID3 + "," + ValUbicacionID4 + "," + ValUbicacionID5 + "," + ValUbicacionID6 + ","
                     + ValUbicacionID7 + "," + ValUbicacionID8 + "," + ValRutId1 + "," + ValRutId2 + "," + ValRutId3 + "," + ValRutId4 + "," + ValRutId5 + "," + ValRutId6 + ","
                     + ValRutId7 + "," + ValRutId8 + "," + ValModoPGM1 + "," + ValModoPGM2 + "," + ValTimPGM1 + "," + ValTimPGM1 + "," + ValTimPGM2 + ","
                     + ValModoCuentaPGM1 + "," + ValModoCuentaPGM2 + "," + ValModoOutAD + "," + ValTimOutAD + ",";
  //              + RamHHTim1 + "," + RamMMTim1 + "," + RamSelDias1 + "," + RamModoAct1 + "," + RamHHTim2 + "," + RamMMTim2 + "," + RamSelDias2 + "," + RamModoAct2 + ","
  //              + RamHHTim3 + "," + RamMMTim3 + "," + RamSelDias3 + "," + RamModoAct3 + "," + RamHHTim4 + "," + RamMMTim4 + "," + RamSelDias4 + "," + RamModoAct4 + ","
  //              + RamHHTim5 + "," + RamMMTim5 + "," + RamSelDias5 + "," + RamModoAct5 + "," + RamHHTim6 + "," + RamMMTim6 + "," + RamSelDias6 + "," + RamModoAct6 + ","
  //              + RamHHTim7 + "," + RamMMTim7 + "," + RamSelDias7 + "," + RamModoAct7 + "," + RamHHTim8 + "," + RamMMTim8 + "," + RamSelDias8 + "," + RamModoAct8 + ",";


  CmdAccion = CmdAccion + resultado;

  Serial.println(CmdAccion);
  CmdBasico = "&from=" + NumSerialESP + "&to=" + IdDestino + "&data=";
  CmdBasico = CmdBasico + CmdAccion;
  MensajeEnviar = CmdBasico;

  //Serial.println("Envio de datos propios : " + MensajeEnviar) + " al usuario que esta subscripto a " + String(IdOrigen) ;
  String DirPublicacion = "";  // = "GN8rYIiMtH9Tt89/output/";
  DirPublicacion = DirPublicacion + String(IdDestino).c_str();
  Serial.println(DirPublicacion);

  MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str(), true);
  Serial.println("Publica al usuario : " + DirPublicacion);     //IdDestino);

  if (MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str()) == true) {
    //if (MQTTClient.publish(String(IdDestino).c_str(), String(MensajeEnviar).c_str()) == true) {
    Serial.println("Envio OK");
    Serial.println("________________________");
  } else {
    Serial.println("Error sending message");
  }
  variable = "";
  DatoRecibido = "";
}



void LeeTimer1() {
  address = E2_TIMER1;
  ValTim1 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim1 = ValTim1.substring(pos, pos + 2);
  RamMMTim1 = ValTim1.substring(pos + 2, pos + 4);
  RamModoAct1 = ValTim1.substring(pos + 4, pos + 5);
  RamSelDias1 = ValTim1.substring(pos + 5, pos + 12);
}
void LeeTimer2() {
  address = E2_TIMER2;
  ValTim2 = "";
  ValTim2 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim2 = ValTim2.substring(pos , pos + 2);
  RamMMTim2 = ValTim2.substring(pos + 2, pos + 4);
  RamModoAct2 = ValTim2.substring(pos + 4, pos + 5);
  RamSelDias2 = ValTim2.substring(pos + 5, pos + 12);
}
void LeeTimer3() {
  address = E2_TIMER3;
  ValTim3 = LeeComunUbicaUsua();
  pos = 0;
  //RamTimMes3 = ValTim3.substring(pos + 0, pos + 2);
  //RamTimDia3 = ValTim3.substring(pos + 2, pos + 4);
  //RamTimAno3 = ValTim3.substring(pos + 4, pos + 8);
  RamHHTim3 = ValTim3.substring(pos , pos + 2);
  RamMMTim3 = ValTim3.substring(pos + 2, pos + 4);
  RamModoAct3 = ValTim3.substring(pos + 4, pos + 5);
  RamSelDias3 = ValTim3.substring(pos + 5, pos + 12);
}
void LeeTimer4() {
  address = E2_TIMER4;
  ValTim4 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim4 = ValTim4.substring(pos , pos + 2);
  RamMMTim4 = ValTim4.substring(pos + 2, pos + 4);
  RamModoAct4 = ValTim4.substring(pos + 4, pos + 5);
  RamSelDias4 = ValTim4.substring(pos + 5, pos + 12);
}
void LeeTimer5() {
  address = E2_TIMER5;
  ValTim5 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim5 = ValTim5.substring(pos + 0, pos + 2);
  RamMMTim5 = ValTim5.substring(pos + 2, pos + 4);
  RamModoAct5 = ValTim5.substring(pos + 4, pos + 5);
  RamSelDias5 = ValTim5.substring(pos + 5, pos + 12);
}
void LeeTimer6() {
  address = E2_TIMER6;
  ValTim6 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim6 = ValTim6.substring(pos + 0, pos + 2);
  RamMMTim6 = ValTim6.substring(pos + 2, pos + 4);
  RamModoAct6 = ValTim6.substring(pos + 4, pos + 5);
  RamSelDias6 = ValTim6.substring(pos + 5, pos + 12);
}
void LeeTimer7() {
  address = E2_TIMER7;
  ValTim7 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim7 = ValTim7.substring(pos + 0, pos + 2);
  RamMMTim7 = ValTim7.substring(pos + 2, pos + 4);
  RamModoAct7 = ValTim7.substring(pos + 4, pos + 5);
  RamSelDias7 = ValTim7.substring(pos + 5, pos + 12);
}
void LeeTimer8() {
  address = E2_TIMER8;
  ValTim8 = LeeComunUbicaUsua();
  pos = 0;
  RamHHTim8 = ValTim8.substring(pos + 0, pos + 2);
  RamMMTim8 = ValTim8.substring(pos + 2, pos + 4);
  RamModoAct8 = ValTim8.substring(pos + 4, pos + 5);
  RamSelDias8 = ValTim8.substring(pos + 5, pos + 12);
}

void EnviaComandoADestino() {
  if (HayInternet == 1) {
    Serial.println("Id Destino : " + IdDestino);
    String DirPublicacion = "";       //GN8rYIiMtH9Tt89/output/";
    DirPublicacion = DirPublicacion + String(IdDestino).c_str();
    Serial.println("Direccion de publicacion : " + DirPublicacion);


    CmdBasico = "&from=" + NumSerialESP + "&to=" + IdDestino + "&data=";
    CmdBasico = CmdBasico + CmdAccion;
    MensajeEnviar = "message" + CmdBasico;
    Serial.println("Texto enviar : " + MensajeEnviar);

    MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str(), true);
    Serial.println("Publica al usuario : " + DirPublicacion);     //IdDestino);

    if (MQTTClient.publish(String(DirPublicacion).c_str(), String(MensajeEnviar).c_str()) == true) {
      Serial.println("Envio OK");
      Serial.println("________________________");
    } else {
      Serial.println("Error sending message");
    }

  } else {
    Serial.println("No puede enviar comando porque no hay internet");
  }
  Serial.println("________________");
}

void Selecciona_RespuestaComun_Notificacion() {
  if (HayInternet == 1)  {
    setSyncProvider(getNtpTime);
    setSyncInterval(300);
    digitalClockDisplay(); //Lee y muestra hORA Y FECHA
    if (EsAccionLocal == 1) {
      EsAccionLocal = 0;
      UsuarioGenerador = "Deteccion interna en el sistema";
      UsuarioGenerador = UsuarioGenerador + "," + ValUbicacionMod;
    } else {
      UsuarioGenerador = UsuarioEmisor + "," + NombreUsuaEmisor;     //IMPORTANTE AQUI DEBE AGREGAR NOMBRE DE USUARIO QUE ENVIO EL PEDIDO DE ACTIVACION
    }
    EnviaNotificacionPush(NumSerialESP);
  }
}

/*
  void SendNotPushHuella(String pTopic) {
  //TopicoDestino="e7j6Ll0odkU:APA91bELKkqOCZxLVolH4sx_MKq7saICxe55HthYdGgRaI68cwgAbt4R6WGUQXp4rhewchf2QYcxoSfAFdsxIOjSNs-I-I7oJ_jrLrnUFxAtUoKhX6ZYXLHzd6BqXS2HtlhIwmBk4Ece";
  //TopicoDestino="cc67XUQDgQI:APA91bHng2Hs7YkPWlJiBagEMVBt96xtzMKF118FgMlgT5Jxod3RxlJ-Woifja9g597_XmVubKo25UMUfbAVWEPnv7MVrdtBoqcBCvSJ2W1w8Nw_Wq4FDzbQOPKW34u9UZxOmRg4Fno6";
  char jsonString[500];
  StaticJsonBuffer<500> jsonBuffer;
  StaticJsonBuffer<200> jsonNotifBuffer;
  jsonNotifBuffer.clear();
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& notif = jsonNotifBuffer.createObject();
  notif["title"] = TituloEvento;
  String strBody = "";
  strBody = strBody + CmdAccion + "\r\n";
  strBody = strBody + ValUbicacionMod + "\r\n";
  strBody = strBody + HoraFechaActual + "\r\n";
  strBody = strBody + IDOrigenEnvio  + "\r\n";
  strBody = strBody + NumSerialESP  + "\r\n";
  notif["body"] = strBody;
  root["data"] = notif;
  root["to"] = "/topics/" + pTopic;
  root["priority"] = 10;
  root.printTo(jsonString);
  Serial.println(jsonString);
  String strRequest = "https://fcm.googleapis.com/fcm/send";
  HTTPClient http;
  http.begin(strRequest, "72:22:AC:55:75:00:2E:B8:87:E1:98:11:60:09:65:6D:71:9C:6D:A0");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "key=AIzaSyBR7G7RZufw1Hafxghz_O3csaXPu4psKI8");
  int httpCode = http.POST(jsonString);
  Serial.println("httpCode=");
  Serial.println(httpCode);
  if (httpCode == -1) {
    Serial.println("No ha podido enviar el codigo debe repetir");
    DebeRepetirEvento = 1;
  } else {
    Serial.println("Ha enviado correctamente la notificacion");
    DebeRepetirEvento = 0;
  }
  if (httpCode) {
    String payload = http.getString();
    Serial.println("Codigo de aceptacion de envio : " + payload);
  }
  http.end();
  }
*/


void BuscaCoincideIdUsua() {
  //Determina di el destino corresponde a un usuario habilitado
  ExisteUsuario = 0;
  if (IdDestino == ValIdReporte1) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte2) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte3) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte4) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte5) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte6) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte7) {
    ExisteUsuario = 1;
  } else if (IdDestino == ValIdReporte8) {
    ExisteUsuario = 1;
  }
}

//&from=131ab0&to=5060141652&data= 131ab0,1234,1111,ubicacion de modulo,,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,00,00,-------,0,
/*
   resultado = variable.substring(pos + 110, pos + 112);
   Serial.println("HH 1: " + resultado);

   resultado = variable.substring(pos + 113, pos + 115);
   Serial.println(resultado);
   Serial.println("MM 1: " + resultado);

   resultado = variable.substring(pos + 117, pos + 125);
   Serial.println(resultado);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 126, pos + 127);
   Serial.println(resultado);
   Serial.println("ACCION 1: " + resultado);
   //17,50,LMNJVSD,1
   address = E2_TIMER1;
   // DEBE GUADAR 12 VALORES!!!
   Serial.println ("___________________________");
   resultado = variable.substring(pos + 128, pos + 130);
   Serial.println("HH 2: " + resultado);

   resultado = variable.substring(pos + 131, pos + 133);
   Serial.println("MM 2: " +  resultado);

   resultado = variable.substring(pos + 134, pos + 141);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 142, pos + 143);
   Serial.println("ACCION 2: " + resultado);
   address = E2_TIMER2;


   Serial.println ("___________________________");
   resultado = variable.substring(pos + 144, pos + 146);
   Serial.println("HH 3: " + resultado);

   resultado = variable.substring(pos + 147, pos + 149);
   Serial.println("MM 2: " +  resultado);

   resultado = variable.substring(pos + 150, pos + 157);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 158, pos + 159);
   Serial.println("ACCION 3: " +  resultado);
   address = E2_TIMER3;

   Serial.println ("___________________________");
   resultado = variable.substring(pos + 160, pos + 162);
   Serial.println("HH 4: " + resultado);

   resultado = variable.substring(pos + 163, pos + 165);
   Serial.println("MM 4: " + resultado);

   resultado = variable.substring(pos + 166, pos + 173);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 174, pos + 175);
   Serial.println("ACCION 4: " +  resultado);
   address = E2_TIMER4;


   Serial.println ("___________________________");
   resultado = variable.substring(pos + 176, pos + 178);
   Serial.println("HH 5: " + resultado);

   resultado = variable.substring(pos + 179, pos + 181);
   Serial.println("MM 5: " +  resultado);

   resultado = variable.substring(pos + 182, pos + 189);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 190, pos + 191);
   Serial.println("ACCION 5: " +  resultado);
   address = E2_TIMER5;

   Serial.println ("___________________________");
   resultado = variable.substring(pos + 192, pos + 194);
   Serial.println("HH 6: " + resultado);

   resultado = variable.substring(pos + 195, pos + 197);
   Serial.println("MM 6: " +  resultado);

   resultado = variable.substring(pos + 198, pos + 205);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 206, pos + 207);
   Serial.println("ACCION 6: " +  resultado);
   address = E2_TIMER6;

   Serial.println ("___________________________");
   resultado = variable.substring(pos + 208, pos + 210);
   Serial.println("HH 7: " + resultado);

   resultado = variable.substring(pos + 211, pos + 213);
   Serial.println("MM 7: " +  resultado);

   resultado = variable.substring(pos + 214, pos + 221);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 222, pos + 223);
   Serial.println("ACCION 7: " +  resultado);
   address = E2_TIMER7;

   Serial.println ("___________________________");
   resultado = variable.substring(pos + 224, pos + 226);
   Serial.println("HH 8: " + resultado);

   resultado = variable.substring(pos + 227, pos + 228);
   Serial.println("MM 8: " + resultado);

   resultado = variable.substring(pos + 229, pos + 216);
   Serial.println("DIAS ACT: " + resultado);

   resultado = variable.substring(pos + 217, pos + 218);
   Serial.println("ACCION 8: " +  resultado);
   address = E2_TIMER8;
*/

//_________________
//Recibe un texto MMQTT
void callback(char *topic, byte * payload, unsigned int length) {
  String incoming = "";
  for (int i = 0; i < length; i++)  {
    incoming += (char)payload[i];
  }
  incoming.trim();
  DatoRecibido = incoming;
  String str_topic(topic);

  Serial.println(topic);
  Serial.println(NumSerialESP);
  Serial.println(DatoRecibido);
  //______________


  if (String(topic) == "PROGMODULO" || String(topic) == DirSubscribe) {

    //Serial.println("Ha entrado en proceso de leer MQTT!");


    // if (String(topic) == "PROGMODULO" || String(topic) == String(NumSerialESP).c_str()) {
    ValorBuscado = "&to=";
    pos = DatoRecibido.indexOf(ValorBuscado);
    if (pos >= 0) {
      resultado = DatoRecibido.substring(pos + 4, pos + 10);
      Serial.println("modulo buscado");
      Serial.println(resultado);
    }
    //genera 2 leds de recepcion

    //Serial.println(NumSerialESP);
    Serial.println(incoming);
    ValorBuscado = "SHARE";
    pos = incoming.indexOf(ValorBuscado);
    if (pos >= 0) {
      //ES SHARE
      EsModoShare = 1;
    } else {
      EsModoShare = 0;
    }
    if (resultado == NumSerialESP) {
      DatoRecibido = incoming;
      //Serial.println("Se va a procesar este mensaje -> " + DatoRecibido);

    } else {
      if (EsModoShare == 1) {
        DatoRecibido = incoming;
        Serial.println("Se va a procesar mensaje por SHARE -> " + DatoRecibido);
      } else {
        Serial.println("No es modulo solicitado");
        DatoRecibido = "";
      }
    }
  }
}
//}

void reconnect() {
  while (!MQTTClient.connected()) {
    //Serial.print("Intentando conexión Mqtt...");
    // Creamos un cliente ID
    //String clientId = "mqttx_9351f744";
    String clientId = "ESP8266Client";
    clientId += String(random(0xffff), HEX);
    // Intentamos conectar
    if (MQTTClient.connect(clientId.c_str(), mqttUser, mqttPass)) {
      Serial.println("Conectado!");
      // Nos suscribimos

      Serial.println(DirSubscribe);
      if (MQTTClient.subscribe(String(DirSubscribe).c_str())) {
        //if (MQTTClient.subscribe(String(NumSerialESP).c_str())) {
        Serial.println("Suscripcion ok");
        //temporal = "PROGMODULO";
        MQTTClient.subscribe("PROGMODULO");                        //String(temporal).c_str());   //subscripcion universal!
        Serial.println("Acaba de subscribirse a PROGRMODULO!");

      } else {
        Serial.println("fallo Suscripción");
      }
    } else {
      Serial.print("falló :( con error -> ");
      Serial.print(MQTTClient.state());
      Serial.println(" Intentamos de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void EnviaNotificacionPush(String pTopic) {

  if (HayUsuario1 == 0 && HayUsuario2 == 0 && HayUsuario3 == 0 && HayUsuario4 == 0 && HayUsuario5 == 0 && HayUsuario6 == 0 && HayUsuario7 == 0 && HayUsuario1 == 0) {
    Serial.println("No va a enviar notificaciones push por no tener usuarios");
  } else {
    Serial.println("[HttpPost]" + pTopic);
    String host  = "fcm.googleapis.com";
    String url   = "/fcm/send";


    //String key   = "AIzaSyBR7G7RZufw1Hafxghz_O3csaXPu4psKI8";
    String key   = "AAAAsUTebDk:APA91bG3ZZnWBraBR96d6Q-EELU89jt-rTXyOgjbwqhvYlKn18q4wK-e-cEWTFza4y2lxJXO86ArM_2BeZvYC9eW9Kb1NWI3xRbt_fXBOXRVJVBqWmLIMJwlGoNZGBEMJiVtQUXW0b_Z";


    String data  = "";

    //-------- JSON CONSTRUCTION --------------------------------------
    StaticJsonDocument<512> doc;
    JsonObject root = doc.to<JsonObject>();
    root["to"]       = "/topics/" + pTopic;
    root["priority"] = 10;

    String strBody = "";
    strBody = strBody + CmdAccion + "\r\n";
    strBody = strBody + ValUbicacionMod + "\r\n";
    strBody = strBody + HoraFechaActual + "\r\n";
    strBody = strBody + UsuarioGenerador  + "\r\n";
    strBody = strBody + NumSerialESP  + "\r\n";
    strBody = strBody + GeneradorPublicacion  + "\r\n";

    //caca3

    JsonObject notif = root.createNestedObject("data");
    notif["title"]   = "Evento en " + ValUbicacionMod;
    notif["body"]    = strBody;
    serializeJson(root, data);
    Serial.print(data);

    //-----------------------------------------------------------------
    WiFiClientSecure client2;
    client2.setInsecure();
    if (client2.connect(host, 443))  {

      Serial.println("");
      Serial.println("Intenta enviar a cliente2");

      client2.println("POST " + url + " HTTP/1.1");
      client2.println("Host: " + (String)host);
      client2.println("User-Agent: ESP8266/1.0");
      client2.println("Connection: close");
      client2.println("Content-Type: application/json");
      client2.println("Authorization: key=" + key);
      client2.print("Content-Length: ");
      client2.println(data.length());
      client2.println();
      client2.println(data);
      delay(20);

      Serial.println("Espera la respuesta al envio a firebase");
      Serial.println(" ");
   
        String response = client2.readString();
        //Serial.println(response);
        int bodypos = response.indexOf("\r\n\r\n") + 4;
        Serial.println(response.substring(bodypos));        //Genera un 0

        //muestra el id de respuesta del envio
        //{"message_id":151923567798637701}
        //muestra un 0
        //hay 4 avances de lineas automatico
  
      Serial.println("_________________");

    } else {

    }
  }
}


void GeneraBeepsLed() {
  for (int i = 0; i < TotalLeds; i++) {
    digitalWrite(PinLedAmarillo, HIGH);    //LOW);
    delay(100);
    digitalWrite(PinLedAmarillo, LOW);    //HIGH);
    delay(100);
  }
}



/*
  {
  "title": string,
  "body": string,
  "icon": string,
  "color": string,
  "sound": string,
  "tag": string,
  "click_action": string,
  "body_loc_key": string,
  "body_loc_args": [
  string
  ],
  "title_loc_key": string,
  "title_loc_args": [
  string
  ],
  "channel_id": string,
  "ticker": string,
  "sticky": boolean,
  "event_time": string,
  "local_only": boolean,
  "notification_priority": enum (NotificationPriority),
  "default_sound": boolean,
  "default_vibrate_timings": booelean,
  "default_light_settings": boolean,
  "vibrate_timings": [
  string
  ],
  "visibility": enum (Visibility),
  "notification_count": integer,
  "light_settings": {
  object (LightSettings)
  },
  "image": string
  }
*/


//Osvaldo = 636f6c6d65
//maria   = 6d61726961,?,?,?,?,
//daniel  = 6e756c6c20

//AAAAsUTebDk:APA91bG3ZZnWBraBR96d6Q-EELU89jt-rTXyOgjbwqhvYlKn18q4wK-e-cEWTFza4y2lxJXO86ArM_2BeZvYC9eW9Kb1NWI3xRbt_fXBOXRVJVBqWmLIMJwlGoNZGBEMJiVtQUXW0b_Z

//[HttpPost]125815
//{"to":"/topics/125815","priority":10,"data":{"title":"Evento en taller","body":"Sistema desarmado\r\ntaller\r\n16:07:58 - 28/08/2020\r\nDeteccion interna en el sistema,taller\r\n125815\r\n\r\n"}}
//Intenta enviar a cliente2
//Espera la respuesta al envio a firebase



//[HttpPost]125815
//{"to":"/topics/125815","priority":10,"data":{"title":"Evento en taller Francia 12581","body":"Activacion salida 1\r\ntaller Francia 12581\r\n18:17:25 - 5/08/2020\r\n6c676e7565,Administrador\r\n125815\r\n\r\n"}}
//22
//{"message_id":7946455438922444421}

//[HttpPost]125815
//{"to":"/topics/125815","priority":10,"data":{"title":"Evento en taller","body":"Sistema desarmado\r\ntaller\r\n16:12:16 - 28/08/2020\r\nDeteccion interna en el sistema,taller\r\n125815\r\n\r\n"}}
//Intenta enviar a cliente2
//Espera la respuesta al envio a firebase
 
//22
//{"message_id":8036871108833771735}
//0
