//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// MACROS DE CONFIGURACION ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define MAXMUESTRAS   699               // Numero maximo de muestras
#define NUMERO_CABLES 1                 // Numero de cables
#define SERIAL0                         // Configuracion de eventos en serial0 (USB)
#define ALARMA                          // Activamos la alarma para salida PIN_RELE_1 y PIN_RELE_2
// #define ALARMA_BOTON_ON                 // Activamos uso de boton para encender
#define ALARMA_BOTON_OFF                // Activamos uso de boton para apagar
#define CONFIG_FILE_NAME "CONF.TXT"     // Nombre del archivo de configuracion
#define LARGO_CHICOTE 0.63              // Largo del chicote
#define RESET_MODBUS false              // Booleano que las variables Modbus se reinician (valor inicial = 0)
#define DEAD_TIME_ARTY 5                // Tiempo de espera de la respuesta de la ARTY


#define TIEMPO_RESET_FALLO_LECTURA_SD 20000 // Tiempo que esperara el sistema antes de realizar un reinicio al fallar la lectura de la SD. En este tiempo el sistema puede hacer ping

// Bytes para comunicacion Arduino-ARTY
#define BYTE_INSTRUCCION_LECTURA 'C'         // Byte que indica inicio de transmision de datos
#define BYTE_INICIO_RECIBIR 'E'         
#define BYTE_FIN_RECIBIR    'X'         // Byte que indica termino de transmision de datos

// Registros por default para los cables
#define DEFAULT_START_REG_CABLE_1   1000
#define DEFAULT_START_REG_CABLE_2   2000
#define DEFAULT_START_REG_CABLE_3   3000
#define DEFAULT_START_REG_CABLE_4   4000
#define DEFAULT_START_REG_CABLE_5   5000

// Registros por default para los cables
#define DEFAULT_START_REG_MEDIAN_FILTER_CABLE_1   6000
#define DEFAULT_START_REG_MEDIAN_FILTER_CABLE_2   7000
#define DEFAULT_START_REG_MEDIAN_FILTER_CABLE_3   8000
#define DEFAULT_START_REG_MEDIAN_FILTER_CABLE_4   9000
#define DEFAULT_START_REG_MEDIAN_FILTER_CABLE_5   10000

// Th1 y th2 por default
#define DEFAULT_TH1_1   1250
#define DEFAULT_TH1_2   1250
#define DEFAULT_TH1_3   1250
#define DEFAULT_TH1_4   1250
#define DEFAULT_TH1_5   1250

#define DEFAULT_TH2_1   2000
#define DEFAULT_TH2_2   2000
#define DEFAULT_TH2_3   2000
#define DEFAULT_TH2_4   2000
#define DEFAULT_TH2_5   2000

#define DEFAULT_TH3_1   3000
#define DEFAULT_TH3_2   3000
#define DEFAULT_TH3_3   3000
#define DEFAULT_TH3_4   3000
#define DEFAULT_TH3_5   3000

#define DEFAULT_IDX_1   35
#define DEFAULT_IDX_2   35
#define DEFAULT_IDX_3   35
#define DEFAULT_IDX_4   35
#define DEFAULT_IDX_5   35

// tiempo de muestreo por defecto
#define DEFAULT_TS      60

// Factor de velocidad por defecto
#define DEFAULT_FACTOR_VELOCIDAD_1 0.63
#define DEFAULT_FACTOR_VELOCIDAD_2 0.63
#define DEFAULT_FACTOR_VELOCIDAD_3 0.63
#define DEFAULT_FACTOR_VELOCIDAD_4 0.63
#define DEFAULT_FACTOR_VELOCIDAD_5 0.63


//----------------------------------------------------------------------------------------------------------------
// Pines //
//----------------------------------------------------------------------------------------------------------------
// Pines luces
#define PIN_ERROR_ETHERNET      27
#define PIN_ERROR_SD            29
#define PIN_ERROR_RTC           31
#define PIN_LECTURA             22

// Pines botones
// #define PIN_ALARMA_BOTON_ON     30
#define PIN_ALARMA_BOTON_OFF    25
// #define PIN_BOTON_CABLE_1       28
// #define PIN_BOTON_CABLE_2       35
// #define PIN_BOTON_CABLE_3       33
// #define PIN_BOTON_CABLE_4       32
// #define PIN_BOTON_CABLE_5       34
// #define PIN_BOTON_CABLES        31

#define PIN_ETHERNET            10
#define PIN_SD                  4
#define PIN_SPI_SS_CODE         53
#define PIN_RELE_1              34
#define PIN_RELE_2              36



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// LIBRERIAS /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <DataLoggerLib.h>
#include <custom_modbus.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include <RTClib_tdr.h>
#include <EEPROM.h>
#include <SDFat.h>
#include <Wire.h>
#include <SPI.h>

#include "medianFilter.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// VARIABLES /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------------------------------------------------
// Variables generales //
//----------------------------------------------------------------------------------------------------------------


uint16_t idArduino      = 0;          // ID del Arduino
bool     estado_general = 0;
bool     estado_rtc     = 0;
bool     estado_sd      = 0;
bool     estado_modbus  = 0;

uint16_t modo_alarma    = 0;          // Modo de alarma: 0 es desactivada, 1 modo normal y 2 silencioso
bool     estado_alarma_fuerte   = 0;          // Estado de la alarma general
bool     estado_alarma_suave    = 0;
uint8_t  tarea        = 1;          // Variable que indica la tarea actual
uint8_t  showMenu     = 1;          // Muestra el menu
uint8_t  Ncable       = 1;          // Numero del cable actual para leer
bool     lecturaAutomatica = 1;     // Habilita la lectura automatica

// Buffer para ir almacenando los datos de la lectura proveniente de la ARTY
uint16_t datosCable[MAXMUESTRAS+ 1 ]; // int array para guardar el valor del cable



// Variables de los cables (Se guardan en un array)

bool      estado_alarma_fuerte_cable  [NUMERO_CABLES]; // bool array para guardar el estado de la alarma
bool      estado_alarma_suave_cable   [NUMERO_CABLES];
uint16_t  estado_alarma_cable         [NUMERO_CABLES];
bool  datosNuevoCable    [NUMERO_CABLES];
float largoInterno      [NUMERO_CABLES];
float largoExterno      [NUMERO_CABLES];
float largoNominal      [NUMERO_CABLES];
float largoMedido       [NUMERO_CABLES];
float umbral_alarma_suave  [NUMERO_CABLES];
float umbral_alarma_fuerte [NUMERO_CABLES];
float factorVelocidad   [NUMERO_CABLES];
uint16_t idx            [NUMERO_CABLES];
uint16_t th1            [NUMERO_CABLES];
uint16_t th2            [NUMERO_CABLES];
uint16_t th3            [NUMERO_CABLES];
uint16_t registrosCables[NUMERO_CABLES];
uint16_t registrosCablesFiltroMediano[NUMERO_CABLES];


// Marcadores (flags) para las distintas tareas.
// flags generales
bool requestRTCLeerCables             = 0;
bool requestLeerCable[NUMERO_CABLES];

// flags de Modbus
bool requestModbusLeerCables              = 0;
bool requestModbusLeerCable[NUMERO_CABLES]   ;
bool requestModbusReset                   = 0;
bool requestModbusGuardarSD               = 0;
bool requestModbusLecturaAutomaticaON     = 0;  
bool requestModbusLecturaAutomaticaOFF    = 0;
bool requestModbusAlarmaON                = 0;
bool requestModbusAlarmaOFF               = 0;
bool requestModbusAlarmaNormal            = 0;
bool requestModbusAlarmaSilenciosa        = 0;
bool requestModbusAlarmaDesactivada       = 0;
bool requestModbusActualizarHora          = 0;

// flags del serial 0
bool requestSerial0LeerCables                = 0;
bool requestSerial0exportData                = 0;
bool requestSerial0LeerCable[NUMERO_CABLES];
bool requestSerial0Reset                     = 0;
bool requestSerial0GuardarSD                 = 0;
bool requestSerial0LecturaAutomaticaON       = 0;
bool requestSerial0LecturaAutomaticaOFF      = 0;
bool requestSerial0AlarmaON                  = 0;
bool requestSerial0AlarmaOFF                 = 0;
bool requestSerial0AlarmaNormal              = 0;
bool requestSerial0AlarmaSilenciosa          = 0;
bool requestSerial0AlarmaDesactivada         = 0;


// flags de Perifericos
#ifdef ALARMA_BOTON_ON
  bool requestBotonON                 = 0;
#endif
#ifdef ALARMA_BOTON_OFF
  bool requestBotonOFF                = 0;
#endif

#ifdef PIN_BOTON_CABLE_1
  bool requestBotonCable1             = 0;
#endif

#ifdef PIN_BOTON_CABLE_2
  bool requestBotonCable2             = 0;
#endif

#ifdef PIN_BOTON_CABLE_3
  bool requestBotonCable3             = 0;
#endif

#ifdef PIN_BOTON_CABLE_4
  bool requestBotonCable4             = 0;
#endif

#ifdef PIN_BOTON_CABLE_5
  bool requestBotonCable5             = 0;
#endif

#ifdef PIN_BOTON_CABLES
  bool requestBotonCables             = 0;
#endif


//----------------------------------------------------------------------------------------------------------------
// RTC y Hora //
//----------------------------------------------------------------------------------------------------------------

// Modulo RTC
RTC_DS3231 RTC;                     // Objeto RTC para modelo DS3231
uint32_t ultima_lectura    = 0;     // Cuando se realizo la ultima lectura
uint32_t rtc_Ts            = 0;     // Variable que indica cada cuanto tiempo se debe realizar una medicion (en segundos)
DateTime datetimeModbus;

//----------------------------------------------------------------------------------------------------------------
// SD y dataLogger //
//----------------------------------------------------------------------------------------------------------------
SdFat Sd;           // Instancia para trabajar con la SD
SdFile MyFile;      // Instancia para manipular datos
dataLogger dataLog; // Objeto DataLogger

//----------------------------------------------------------------------------------------------------------------
//Variables para Ethernet-Modbus //
//----------------------------------------------------------------------------------------------------------------

// Variables para conexion ethernet
uint8_t server   [4];
uint8_t mac      [6];
uint8_t ip       [4];
uint8_t dnServer [4];
uint8_t gateway  [4];
uint8_t subnet   [4];

// Objeto modbus
Modbus modbus;

// Objeto para filtrar
medianFilter mFilter[NUMERO_CABLES];

// Otros
union float2bytes{float f; uint16_t b[2];};
union float2bytes f2b;


unsigned long tiempo_ultimo_status;


// Constructor de las funciones
void setup_serial(void);
void setup_reset(void);
void setup_alarma(void);
void setup_rtc(void);
void setup_SD(void);
void setup_leer_Config_SD(void);
void error_lectura_SD(void);
void guardar_datos_eeprom(void);
void leer_datos_eeprom(void);
void setup_ethernet(void);
void setup_dataLogger(void);
void setup_modbus(void);
void setup_median_filter(void);
void setup_botones(void);
void revisar_estado(void);
bool revisar_estado_modbus(void);
bool revisar_estado_sd(void);
void modbus_var2reg(void);
void modbus_reg2var(void);
void Rx_request_serial0(void);
void Rx_request_perifericos(void);
void Rx_request_RTC(void);
void resolver_request(void);
void exportarDatosSerial(void);
void leerCable(uint8_t Ncable);
float calcularLargoCable(uint8_t Ncable, uint16_t N_muestras);
uint16_t definirMuestras(uint16_t largo);
bool recibirDatosSerial1(void);
void guardarDatosModbus(uint8_t Ncable, uint16_t N_muestras);
void guardarDatosFiltradosModbus(uint8_t Ncable, uint16_t N_muestras);
void guardarDatosSD(uint8_t Ncable, uint16_t N_muestras);
void guardarDatosFiltradosSD(uint8_t Ncable, uint16_t N_muestras);
void guardarDatosArchivo(uint8_t Ncable, uint16_t N_muestras);
void actualizar_hora(void);
void statusAlarma(void);
void revisarAlarma(uint8_t Ncable);
void alarma_ON(void);
void alarma_OFF(void);
void actualizar_Config_SD(void);
void remove_last_char(char *d, const char *s);
void remove_space(char *d, const char *s);
void split_array(char *var, char *val, const char*s);
int array_point_counter(const char *s);
void split_ip_array(char *s, uint8_t *ip);





void setup() {
  pinMode(PIN_LECTURA, OUTPUT);
  digitalWrite(PIN_LECTURA, HIGH);
  for(register int i=0; i<NUMERO_CABLES; i++){
    registrosCables[i] = 0;
    registrosCablesFiltroMediano[i] = 0;
    idx[i] = 0;
    th1[i] = 0;
    th2[i] = 0;
    th3[i] = 0;
    factorVelocidad[i] = 0.0;
  }
  tiempo_ultimo_status = millis();
  // Iniciamos los distintos modulos
  pinMode(A15, OUTPUT);
  digitalWrite(A15, HIGH);
  
  // Inicio de funciones internas del ATMEGA
  setup_serial();         // Activamos y configuramos comunicaciones seriales
  setup_reset();          // Activamos y configuramos el registro para el reiniciar el sistema
  setup_alarma();         // Activamos y configuramos la alarma
  setup_rtc();            // Activamos y configuramos el RTC
  
  // Configuramos los SS para SPI
  pinMode(PIN_SPI_SS_CODE, OUTPUT);
  digitalWrite(PIN_SPI_SS_CODE, HIGH);

  pinMode(PIN_ETHERNET, OUTPUT);
  digitalWrite(PIN_ETHERNET, HIGH);

  pinMode(PIN_SD, OUTPUT);
  digitalWrite(PIN_SD, HIGH);

  SPI.begin();

  setup_SD();             // Activamos y configuramos la SD
  setup_leer_Config_SD(); // Leemos los datos de configuracion de la SD

  // Con los datos medidos de la SD, calculamos los largos nominales
  for (int i=0; i<NUMERO_CABLES; i++){
    largoNominal[i] = largoInterno[i] + largoExterno[i];
  }
  
  setup_ethernet();       // Activamos y configuramos el Ethernet
  // Inicio de funciones de modulos de alto nivel
  setup_dataLogger();     // Activamos y configuramos la clase dataLogger
  setup_modbus();         // Activamos y configuramos modbus
  setup_median_filter();
  setup_botones();        // Activamos los botones
  
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("-        Setup finalizado          -"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
  #endif

  for(int i=0; i<3; i++){
    digitalWrite(PIN_LECTURA, LOW);
    delay(150);
    digitalWrite(PIN_LECTURA, HIGH);
    delay(150);
  }
  digitalWrite(PIN_LECTURA, LOW);
  delay(150);
}


void loop() {
  bool lect_prev = lecturaAutomatica;
  
  // Actualizamos Modbus
  modbus_var2reg();   // Pasamos los datos desde las variables locales a registros modbus
  modbus.run();       // Actualizamos los registros Modbus
  modbus_reg2var();   // Pasamos los datos desde los registros de modbus a variables locales

  // En caso de que la lectura automatica se haya activado, se guarda la hora
  if ((lecturaAutomatica == 1) && (lect_prev == 0)){
    // Actualizamos la ultima lectura
    DateTime fecha = RTC.now();             // Leemos la hora del RTC
    ultima_lectura = fecha.secondstime();   // Extraemos el tiempo actual
  }
  
  // Actualiamos el largo de los cables nominales
  for (int i=0; i<NUMERO_CABLES; i++){
    largoNominal[i] = largoInterno[i] + largoExterno[i];
  }
  // Chequeamos el estado de las distintas partes del programa
  statusAlarma();

  
  if(tiempo_ultimo_status + 1000 < millis())
  {
    revisar_estado();
    tiempo_ultimo_status = millis();
  }

  // Revisamos posibles request
  Rx_request_serial0();       // Request del serial 0
  Rx_request_perifericos();   // Request de los perifericos (botones)
  Rx_request_RTC();           // Request del RTC
  // Modbus activa los request directamente en los request
  
  // Revisamos los hay request disponibles
  resolver_request();
}



//////////////////////////////////////////////////////////////////
///                   - FUNCIONES DE INICIO -               //////
//////////////////////////////////////////////////////////////////


/*
 * Funcion que activa el puerto serial 0 y 1
 */
void setup_serial(void) {
  // Iniciamos la comunicacion serial de cada puerto
  
  // Serial 0: USB A PC
  Serial.begin(115200);
  while (!Serial) {
    ; // Esperamos a que el puerto serial se conecte
    }
  
  // Serial a ARTY
  Serial1.begin(19200);
  while (!Serial1) {
    ; // Esperamos a que el puerto serial se conecte
  }

  #ifdef SERIAL0
    Serial.println(F(""));
    Serial.println(F("------------------------------------"));
    Serial.println(F("-----      Sistema TDR         -----"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
    Serial.println(F("------------------------------------"));
    Serial.println(F("-        Setup Iniciado            -"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup Seriales                   -"));
    Serial.println(F("------------------------------------"));
    Serial.println(F("- COM 0     : OK"));
    Serial.println(F("- COM 1     : OK"));
    Serial.println(F(""));
  #endif
}


/*
 * Funcion que activa el reset
 * De momento se utiliza la funcion del watchdog
 */
void setup_reset(void) {
  MCUSR = 0;      // Registro utilizado para usar un "Reset" en la programacion
}


/*
 *  Funcion que inicializa la alarma
 */
void setup_alarma(void){
  // Si el macro ALARMA esta definido activamos la alarma
  #ifdef ALARMA
    // Configuramos los pines de la alarma como salida
    pinMode(PIN_RELE_1, OUTPUT);
    digitalWrite(PIN_RELE_1, HIGH);
    pinMode(PIN_RELE_2, OUTPUT);
    digitalWrite(PIN_RELE_2, HIGH);
    // Configuramos los pines de los botones como entrada en el caso de que los macros esten definidos

    
    #ifdef ALARMA_BOTON_ON
      pinMode(PIN_ALARMA_BOTON_ON, INPUT_PULLUP);
    #endif
    #ifdef ALARMA_BOTON_OFF
      pinMode(PIN_ALARMA_BOTON_OFF, INPUT_PULLUP);
    #endif
    
    // Apagamos la alarma
    alarma_OFF();
    #ifdef SERIAL0
      Serial.println(F(""));
      Serial.println(F("------------------------------------"));
      Serial.println(F("- Setup Alarma                     -"));
      Serial.println(F("------------------------------------"));
      Serial.println(F("- Alarma Configurada"));
      #ifdef ALARMA_BOTON_ON
        Serial.println(F(" -> Boton ON"));
      #endif
      #ifdef ALARMA_BOTON_OFF
        Serial.println(F(" -> Boton OFF"));
      #endif
    #endif
  #endif
}


/*
 * Funcion que inicializa el RTC.
 */
void setup_rtc(void) {
  
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup Modulo RTC                 -"));
    Serial.println(F("------------------------------------"));
  #endif

  // Configuramos el pin para indicar error
  pinMode(PIN_ERROR_RTC, OUTPUT);
  digitalWrite(PIN_ERROR_RTC, LOW);
  
  // Inicializamos el RTC
  if (!RTC.begin()) {
    // En caso de encontrar un error, lo indicamos en el pin de salida
    digitalWrite(PIN_ERROR_RTC, HIGH);
    #ifdef SERIAL0
      Serial.println(F("- No se puede encontrar el RTC"));
    #endif
    // Dado que no se pudo iniciar el RTC, se termina la funcion
    return;
  }

  // Si el RTC tuvo perdida de energia configura la hora
  if (RTC.lostPower()) {
    #ifdef SERIAL0
      Serial.println(F("- RTC con perdida de energia: Ajustando fecha y hora"));
    #endif
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Si el modo de prueba esta activo, se generan los mensajes
  DateTime datetime = RTC.now();
  ultima_lectura = datetime.secondstime();
  #ifdef SERIAL0
    Serial.println(F("- RTC       : OK"));
    char str[20];
    sprintf(str, "%d-%02d-%02d %02d:%02d:%02d", 
    datetime.year(), 
    datetime.month(),
    datetime.day(), 
    datetime.hour(), 
    datetime.minute(), 
    datetime.second()
    );
    Serial.print  (F("- Hora      : ")); Serial.println(str);
    Serial.println(F(""));
  #endif
}


/*
 *  Funcion que inicia la SD
 */
void setup_SD(void) {
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup Modulo SD                  -"));
    Serial.println(F("------------------------------------"));
    Serial.print(F("- Iniciando SD . . . "));
  #endif
  
  // Configuramos los pines de salida modulo SD y Ethernet mas selector de error
  pinMode(PIN_ERROR_SD, OUTPUT);
  pinMode(PIN_ERROR_ETHERNET, OUTPUT);

  // Damos valores iniciales a dichos pines
  digitalWrite(PIN_ERROR_SD, LOW);        // SD sin error
  digitalWrite(PIN_ERROR_ETHERNET, LOW);  // Ethernet sin error
  
  // Iniciamos la SD
  digitalWrite(PIN_ETHERNET, HIGH);
  digitalWrite(PIN_SD, LOW);
  if (!Sd.begin(PIN_SD, SPI_FULL_SPEED)){
    // En caso de encontrar un error, lo indicamos en el pin de salida
    digitalWrite(PIN_ERROR_SD, HIGH);
    digitalWrite(PIN_SD, HIGH);
    #ifdef SERIAL0
      Serial.println(F("No se pudo encontrar la SD"));
      Serial.println(F(""));
    #endif
    // Dado que no se pudo iniciar la SD, se termina la funcion
    return;
  }
  #ifdef SERIAL0
    Serial.println(F("Ok!"));
    Serial.println(F(""));
  #endif
  digitalWrite(PIN_SD, HIGH);
}


/*
 * Funcion que lee el archivo de configuracion
 */
void setup_leer_Config_SD(void){
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup lectura config SD           -"));
    Serial.println(F("------------------------------------"));
    Serial.print(F("- Abriendo archivo config . . . "));
  #endif
  // Creamos y abrimos el archivo
  if (!MyFile.open(CONFIG_FILE_NAME, O_READ)){
    digitalWrite(PIN_ERROR_SD, HIGH);
    #ifdef SERIAL0
      Serial.println(F("- No se pudo encontrar el archivo de configuracion"));
      Serial.println(F(""));
    #endif
    // Dado que no se pudo iniciar el archivo, se inicia a a una rutina especifica
    error_lectura_SD();
    return;
  }
  #ifdef SERIAL0
    Serial.println(F("Ok!"));  
    Serial.println(F("- leyendo archivo config . . ."));
  #endif
    // Creamos char array para leer los datos
  char buff1[100];    // Primer Array para leer los datos
  char buff2[100];    // Segundo Array para leer los datos
  char value[50];     // Array para guardar el valor de la variable

  // Mientras hayan datos en el archivo
  while(MyFile.available()){
    // Leemos una linea del archivo en buff1
    char s[] = "\n";
    MyFile.fgets(buff1, sizeof(buff1), s);
    
    remove_last_char(buff2, buff1); // Removemos el ultimo caracter '\n'
    remove_space(buff1, buff2);     // Removemos los espacios

    // Si el char array parte con // o NULL se ignora
    if ((buff1[0] != '/') && (buff1[1] != '/') && buff1[0] != '\0'){
      #ifdef SERIAL0
        Serial.print(F(" -> '")); Serial.print(buff1);  Serial.println(F("'"));
      #endif

      // Dividimos el array en el nombre de la variable (guardado en buff2) y en value
      split_array(buff2, value, buff1);

      // Variables principales
      if      (strcmp(buff2, "idAr")        == 0)  idArduino   = atoi(value);
      else if (strcmp(buff2, "Ts")          == 0)  rtc_Ts      = atoi(value);
      else if (strcmp(buff2, "modo_alarma") == 0)  modo_alarma = atoi(value);

      // Configuracion del servidor
      else if (strcmp(buff2, "serv") == 0)  split_ip_array(value, server);
      else if (strcmp(buff2, "myip") == 0)  split_ip_array(value, ip);
      else if (strcmp(buff2, "dnSe") == 0)  split_ip_array(value, dnServer);
      else if (strcmp(buff2, "gate") == 0)  split_ip_array(value, gateway);
      else if (strcmp(buff2, "mask") == 0)  split_ip_array(value, subnet);
      else if (strcmp(buff2, "mac_") == 0){
        char buff3[3];
        buff3[2] = '\0';
        for (int i=0; i<6; i++){
          buff3[0] = value[2*i];
          buff3[1] = value[2*i + 1];
          mac[i] = strtol(buff3, NULL, 16);
        }
      }
      
      // Datos por cable
      else if (strcmp(buff2, "LargoExterno1") == 0)  largoExterno[0] = atof(value);
      else if (strcmp(buff2, "LargoInterno1") == 0)  largoInterno[0] = atof(value);
      else if (strcmp(buff2, "umbralAlarmaFuerte1") == 0)  umbral_alarma_fuerte[0] = atof(value);
      else if (strcmp(buff2, "umbralAlarmaSuave1") == 0)  umbral_alarma_suave[0] = atof(value);
      else if (strcmp(buff2, "factorVelocidad1") == 0)  factorVelocidad[0] = atof(value);
      else if (strcmp(buff2, "idx_1") == 0)  idx[0] = atoi(value);
      else if (strcmp(buff2, "th1_1") == 0)  th1[0] = atoi(value);
      else if (strcmp(buff2, "th1_2") == 0)  th2[0] = atoi(value);
      else if (strcmp(buff2, "th1_3") == 0)  th3[0] = atoi(value);
      else if (strcmp(buff2, "start_reg_cable_1") == 0)  registrosCables[0] = atoi(value);
      else if (strcmp(buff2, "start_reg_cable_fil_1") == 0)  registrosCablesFiltroMediano[0] = atoi(value);
    }
  }
  MyFile.close();

  // Generamos los datos por default en caso de no encontrarlos en la SD

  if (rtc_Ts == 0) rtc_Ts = DEFAULT_TS;

  if (factorVelocidad[0] == 0.0) factorVelocidad[0] = DEFAULT_FACTOR_VELOCIDAD_1;
  
  if (idx[0] == 0) idx[0] = DEFAULT_IDX_1;
  if (th1[0] == 0) th1[0] = DEFAULT_TH1_1;
  if (th2[0] == 0) th2[0] = DEFAULT_TH2_1;
  if (th3[0] == 0) th3[0] = DEFAULT_TH3_1;

  if (registrosCables[0] == 0) registrosCables[0] = DEFAULT_START_REG_CABLE_1;
  if (registrosCablesFiltroMediano[0] == 0) registrosCablesFiltroMediano[0] = DEFAULT_START_REG_MEDIAN_FILTER_CABLE_1;

  #ifdef SERIAL0
    Serial.println(F("- Fin de lectura de datos"));
    Serial.println(F(""));
    Serial.println(F("- Datos leidos: "));  
    
    Serial.print(F(" -> idAr:        "));   Serial.println(idArduino);
    Serial.print(F(" -> Ts:          "));   Serial.println(rtc_Ts);
    Serial.print(F(" -> modo alarma: "));   Serial.println(modo_alarma);
    Serial.println(F("   (0: apagada, 1: normal, 2: silenciosa)"));

    Serial.print(F(" -> Serv: "));     
    Serial.print(server[0]);  Serial.print(F("."));
    Serial.print(server[1]);  Serial.print(F("."));
    Serial.print(server[2]);  Serial.print(F("."));
    Serial.print(server[3]);  Serial.println(F(""));
    Serial.print(F(" -> Mac:  "));     
    Serial.print(mac[0], HEX);  Serial.print(F(" "));
    Serial.print(mac[1], HEX);  Serial.print(F(" "));
    Serial.print(mac[2], HEX);  Serial.print(F(" "));
    Serial.print(mac[3], HEX);  Serial.print(F(" "));
    Serial.print(mac[4], HEX);  Serial.print(F(" "));
    Serial.print(mac[5], HEX);  Serial.println(F(""));
    Serial.print(F(" -> myip: "));     
    Serial.print(ip[0]);  Serial.print(F("."));
    Serial.print(ip[1]);  Serial.print(F("."));
    Serial.print(ip[2]);  Serial.print(F("."));
    Serial.print(ip[3]);  Serial.println(F(""));
    Serial.print(F(" -> DnSe: "));     
    Serial.print(dnServer[0]);  Serial.print(F("."));
    Serial.print(dnServer[1]);  Serial.print(F("."));
    Serial.print(dnServer[2]);  Serial.print(F("."));
    Serial.print(dnServer[3]);  Serial.println(F(""));
    Serial.print(F(" -> gate: "));     
    Serial.print(gateway[0]);  Serial.print(F("."));
    Serial.print(gateway[1]);  Serial.print(F("."));
    Serial.print(gateway[2]);  Serial.print(F("."));
    Serial.print(gateway[3]);  Serial.println(F(""));
    Serial.print(F(" -> mask: "));     
    Serial.print(subnet[0]);  Serial.print(F("."));
    Serial.print(subnet[1]);  Serial.print(F("."));
    Serial.print(subnet[2]);  Serial.print(F("."));
    Serial.print(subnet[3]);  Serial.println(F(""));
    
    Serial.print(F(" -> LargoExterno1: "));  Serial.println(largoExterno[0]);
    Serial.print(F(" -> LargoInterno1: "));  Serial.println(largoInterno[0]);
    Serial.print(F(" -> umbralAlarmaFuerte1: "));  Serial.println(umbral_alarma_fuerte[0]);
    Serial.print(F(" -> umbralAlarmaSuave1: "));  Serial.println(umbral_alarma_suave[0]);
    Serial.print(F(" -> factorVelocidad1: "));  Serial.println(factorVelocidad[0]);
    Serial.print(F(" -> idx1: "));  Serial.println(idx[0]);
    Serial.print(F(" -> th1_1: "));  Serial.println(th1[0]);
    Serial.print(F(" -> th1_2: "));  Serial.println(th2[0]);
    Serial.print(F(" -> th1_3: "));  Serial.println(th3[0]);
    Serial.print(F(" -> registro inicial cable 1: "));  Serial.println(registrosCables[0]);
    Serial.print(F(" -> registro inicial cable 1 con filtro: "));  Serial.println(registrosCablesFiltroMediano[0]);
  #endif
  // Guardamos los datos leidos en la eeprom
  guardar_datos_eeprom();
}


void error_lectura_SD(void){
  // Leemos los datos de la SD
  leer_datos_eeprom();
  #ifdef SERIAL0
    Serial.println(F("- Rutina de fallo de lectura de la SD"));
    Serial.println(F(" -> Iniciamos el modulo ethernet con los datos de la EEPROM"));
  #endif
  // Iniciamos el modulo ethernet para que sea posible hacer ping
  setup_ethernet();

  #ifdef SERIAL0
    Serial.println(F(" -> Esperando un tiempo antes de realizar un reinicio al sistema"));
  #endif
  // realizamos un loop de reinicio
  unsigned long t_lectura = millis();
  while(millis() < TIEMPO_RESET_FALLO_LECTURA_SD + t_lectura) {
  ;
  // LOOP DE ERROR //
  }
  
  #ifdef SERIAL0
    Serial.println(F(" -> Reiniciando el sistema"));
  #endif
  wdt_enable(WDTO_15MS);
  for (;;) {}
  return;
}


void guardar_datos_eeprom(void){
  #ifdef SERIAL0
    Serial.print(F("- Guardando datos en la EEPROM . . ."));
  #endif
  EEPROM.put(0, idArduino);
  EEPROM.put(2, modo_alarma);
  EEPROM.put(5, rtc_Ts);
  
  EEPROM.put(10, server[0]);
  EEPROM.put(11, server[1]);
  EEPROM.put(12, server[2]);
  EEPROM.put(13, server[3]);
   
  EEPROM.put(14, mac[0]);
  EEPROM.put(15, mac[1]);
  EEPROM.put(16, mac[2]);
  EEPROM.put(17, mac[3]);
  EEPROM.put(18, mac[4]);
  EEPROM.put(19, mac[5]);
      
  EEPROM.put(20, ip[0]);
  EEPROM.put(21, ip[1]);
  EEPROM.put(22, ip[2]);
  EEPROM.put(23, ip[3]);
    
  EEPROM.put(24, dnServer[0]);
  EEPROM.put(25, dnServer[1]);
  EEPROM.put(26, dnServer[2]);
  EEPROM.put(27, dnServer[3]);
  
  EEPROM.put(28, gateway[0]);
  EEPROM.put(29, gateway[1]);
  EEPROM.put(30, gateway[2]);
  EEPROM.put(31, gateway[3]);
       
  EEPROM.put(32, subnet[0]);
  EEPROM.put(33, subnet[1]);
  EEPROM.put(34, subnet[2]);
  EEPROM.put(35, subnet[3]);
  
  EEPROM.put(40, largoExterno[0]);
  EEPROM.put(65, largoInterno[0]);
  EEPROM.put(90,  umbral_alarma_fuerte[0]);
  EEPROM.put(115, umbral_alarma_suave[0]);
  EEPROM.put(140, factorVelocidad[0]);
  EEPROM.put(170, idx[0]);
  EEPROM.put(180, th1[0]);
  EEPROM.put(190, th2[0]);
  EEPROM.put(200, th3[0]);
  EEPROM.put(210, registrosCables[0]);
  EEPROM.put(220, registrosCablesFiltroMediano[0]);

  #ifdef SERIAL0
    Serial.println(F(" ok!"));
  #endif
}


void leer_datos_eeprom(void){
  #ifdef SERIAL0
    Serial.print(F("- leyendo datos en la EEPROM . . ."));
  #endif
  EEPROM.get(0, idArduino);
  EEPROM.get(2, modo_alarma);
  EEPROM.get(5, rtc_Ts);
  
  EEPROM.get(10, server[0]);
  EEPROM.get(11, server[1]);
  EEPROM.get(12, server[2]);
  EEPROM.get(13, server[3]);
   
  EEPROM.get(14, mac[0]);
  EEPROM.get(15, mac[1]);
  EEPROM.get(16, mac[2]);
  EEPROM.get(17, mac[3]);
  EEPROM.get(18, mac[4]);
  EEPROM.get(19, mac[5]);
      
  EEPROM.get(20, ip[0]);
  EEPROM.get(21, ip[1]);
  EEPROM.get(22, ip[2]);
  EEPROM.get(23, ip[3]);
    
  EEPROM.get(24, dnServer[0]);
  EEPROM.get(25, dnServer[1]);
  EEPROM.get(26, dnServer[2]);
  EEPROM.get(27, dnServer[3]);
  
  EEPROM.get(28, gateway[0]);
  EEPROM.get(29, gateway[1]);
  EEPROM.get(30, gateway[2]);
  EEPROM.get(31, gateway[3]);
       
  EEPROM.get(32, subnet[0]);
  EEPROM.get(33, subnet[1]);
  EEPROM.get(34, subnet[2]);
  EEPROM.get(35, subnet[3]);
  
  EEPROM.get(40, largoExterno[0]);
  EEPROM.get(65, largoInterno[0]);
  EEPROM.get(90,  umbral_alarma_fuerte[0]);
  EEPROM.get(115, umbral_alarma_suave[0]);
  EEPROM.get(140, factorVelocidad[0]);
  EEPROM.get(170, idx[0]);
  EEPROM.get(180, th1[0]);
  EEPROM.get(190, th2[0]);
  EEPROM.get(200, th3[0]);
  EEPROM.get(210, registrosCables[0]);
  EEPROM.get(220, registrosCablesFiltroMediano[0]);

  #ifdef SERIAL0
    Serial.println(F(" ok!"));
    Serial.println(F("- Datos leidos: "));  
    
    Serial.print(F(" -> idAr:        "));   Serial.println(idArduino);
    Serial.print(F(" -> Ts:          "));   Serial.println(rtc_Ts);
    Serial.print(F(" -> modo alarma: "));   Serial.println(modo_alarma);
    Serial.println(F("   (0: apagada, 1: normal, 2: silenciosa)"));

    Serial.print(F(" -> Serv: "));     
    Serial.print(server[0]);  Serial.print(F("."));
    Serial.print(server[1]);  Serial.print(F("."));
    Serial.print(server[2]);  Serial.print(F("."));
    Serial.print(server[3]);  Serial.println(F(""));
    Serial.print(F(" -> Mac:  "));     
    Serial.print(mac[0], HEX);  Serial.print(F(" "));
    Serial.print(mac[1], HEX);  Serial.print(F(" "));
    Serial.print(mac[2], HEX);  Serial.print(F(" "));
    Serial.print(mac[3], HEX);  Serial.print(F(" "));
    Serial.print(mac[4], HEX);  Serial.print(F(" "));
    Serial.print(mac[5], HEX);  Serial.println(F(""));
    Serial.print(F(" -> myip: "));     
    Serial.print(ip[0]);  Serial.print(F("."));
    Serial.print(ip[1]);  Serial.print(F("."));
    Serial.print(ip[2]);  Serial.print(F("."));
    Serial.print(ip[3]);  Serial.println(F(""));
    Serial.print(F(" -> DnSe: "));     
    Serial.print(dnServer[0]);  Serial.print(F("."));
    Serial.print(dnServer[1]);  Serial.print(F("."));
    Serial.print(dnServer[2]);  Serial.print(F("."));
    Serial.print(dnServer[3]);  Serial.println(F(""));
    Serial.print(F(" -> gate: "));     
    Serial.print(gateway[0]);  Serial.print(F("."));
    Serial.print(gateway[1]);  Serial.print(F("."));
    Serial.print(gateway[2]);  Serial.print(F("."));
    Serial.print(gateway[3]);  Serial.println(F(""));
    Serial.print(F(" -> mask: "));     
    Serial.print(subnet[0]);  Serial.print(F("."));
    Serial.print(subnet[1]);  Serial.print(F("."));
    Serial.print(subnet[2]);  Serial.print(F("."));
    Serial.print(subnet[3]);  Serial.println(F(""));
    Serial.print(F(" -> LargoExterno1: "));  Serial.println(largoExterno[0]);
    Serial.print(F(" -> LargoInterno1: "));  Serial.println(largoInterno[0]);
    Serial.print(F(" -> umbralAlarmaFuerte1: "));  Serial.println(umbral_alarma_fuerte[0]);
    Serial.print(F(" -> umbralAlarmaSuave1: "));  Serial.println(umbral_alarma_suave[0]);
    Serial.print(F(" -> factorVelocidad1: "));  Serial.println(factorVelocidad[0]);
    Serial.print(F(" -> th1_1: "));  Serial.println(th1[0]);
    Serial.print(F(" -> th1_2: "));  Serial.println(th2[0]);
    Serial.print(F(" -> registro inicial cable 1: "));  Serial.println(registrosCables[0]);
    Serial.print(F(" -> registro inicial cable 1 con filtro: "));  Serial.println(registrosCablesFiltroMediano[0]);
  #endif
}


/*
 * Funcion para inicializar el modulo Ethernet
 */
void setup_ethernet(void){
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup Ethernet                   -"));
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Variables de configuracion"));
    Serial.print(F(" -> serv: ")); 
    for (int i = 0; i < 4; i++) {Serial.print(server[i]); if(i != 3) Serial.print(F("."));}
    Serial.println(F(""));

    Serial.print(F(" -> mac_: ")); 
    for (int i = 0; i < 6; i++) {Serial.print(mac[i], HEX); Serial.print(F(" "));}
    Serial.println();
    
    Serial.print(F(" -> myip: "));
    for (int i = 0; i < 4; i++) {Serial.print(ip[i]); if (i != 3) Serial.print(F("."));} 
    Serial.println();

    Serial.print(F(" -> dnse: "));
    for (int i = 0; i < 4; i++) {Serial.print(dnServer[i]); if (i != 3) Serial.print(F("."));} 
    Serial.println();
    
    Serial.print(F(" -> gate: "));
    for (int i = 0; i < 4; i++) {Serial.print(gateway[i]); if (i != 3) Serial.print(F("."));}
    Serial.println();
    
    Serial.print(F(" -> mask: "));
    for (int i = 0; i < 4; i++) {Serial.print(subnet[i]); if (i != 3) Serial.print(F("."));} 
    Serial.println();
  #endif
  
  // Iniciamos la comunicacion ethernet
  Ethernet.begin(mac, ip, dnServer, gateway, subnet);
  
  #ifdef SERIAL0
    Serial.println(F("- Modulo Ethernet   : OK"));
    Serial.println(F(""));
  #endif
}


/*
 * Funcion que inicializa el dataLogger
 */
void setup_dataLogger(void) {
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup DataLogger                 -"));
    Serial.println(F("------------------------------------"));
    Serial.print(F("- Creando carpetas . . . "));
  #endif
  // Creamos las carpetas
  char file_name[8];
  for(int i=0; i<NUMERO_CABLES; i++){
    sprintf(file_name, "Cable_%d", i+1);
    dataLog.changeFolderPath(file_name);
    }
  #ifdef SERIAL0
    Serial.println(F("Ok!"));
    Serial.println(F(""));
  #endif
}


/*
 * Funcion que inicializa Modbus
 */
void setup_modbus(void) {
  
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.println(F("- Setup Modbus                     -"));
    Serial.println(F("------------------------------------"));
    Serial.print(F("- Creando registros Modbus . . . "));
  #endif
  
  // Iniciamos Modbus
  if (!modbus.begin(false)){
    // En caso de encontrar un error, lo indicamos en el pin de salida
    digitalWrite(PIN_ERROR_ETHERNET, HIGH);
    #ifdef SERIAL0
      Serial.println(F("No se pudo iniciar Modbus"));
      Serial.println(F(""));
    #endif
    return;
  }
  // Definimos los request como cero
  for (int i=0; i<NUMERO_CABLES; i++) requestModbusLeerCable[i] = 0;
  
  #ifdef SERIAL0
    Serial.println(F("Ok!"));
    Serial.println(F("- Comunicacion Modbus iniciada"));
    Serial.println(F(""));
  #endif
}


void setup_median_filter(void){
  char s_buff[30];
  for(int i=0; i<NUMERO_CABLES; i++){
    sprintf(s_buff, "filtro_cable_%d.csv", i + 1);
    strcpy(mFilter[i].fileName, s_buff);  
    mFilter[i].create_file();
  }
}


/*
 * Funcion que inicializa los botones
 */
void setup_botones(void) {
  // Si estan definidos los pines de los botones, se configuran como input con pull-up
  #ifdef PIN_BOTON_CABLE_1
    pinMode(PIN_BOTON_CABLE_1, INPUT_PULLUP);
  #endif
  
  #ifdef PIN_BOTON_CABLE_2
    pinMode(PIN_BOTON_CABLE_2, INPUT_PULLUP);
  #endif
  
  #ifdef PIN_BOTON_CABLE_3
    pinMode(PIN_BOTON_CABLE_3, INPUT_PULLUP);
  #endif
  
  #ifdef PIN_BOTON_CABLE_4
    pinMode(PIN_BOTON_CABLE_4, INPUT_PULLUP);
  #endif
  
  #ifdef PIN_BOTON_CABLE_5
    pinMode(PIN_BOTON_CABLE_5, INPUT_PULLUP);
  #endif
  
  #ifdef PIN_BOTON_CABLES
    pinMode(PIN_BOTON_CABLES, INPUT_PULLUP);
  #endif
}


//////////////////////////////////////////////////////////////////
///         - FUNCIONES DE ESTADO -                         //////
//////////////////////////////////////////////////////////////////

void revisar_estado(void){
  estado_rtc = (bool) RTC.status();
  if (!estado_rtc) digitalWrite(PIN_ERROR_RTC, LOW);
  else digitalWrite(PIN_ERROR_RTC, HIGH);

  estado_sd      = revisar_estado_sd();
  if (!estado_sd) digitalWrite(PIN_ERROR_SD, LOW);
  else digitalWrite(PIN_ERROR_SD, HIGH);
  
  estado_modbus  = revisar_estado_modbus();
  if (!estado_modbus) digitalWrite(PIN_ERROR_ETHERNET, LOW);
  else digitalWrite(PIN_ERROR_ETHERNET, HIGH);
  
  estado_general = estado_rtc || estado_sd || estado_modbus;
}


bool revisar_estado_modbus(void){
  digitalWrite(SD_PIN, LOW);
  digitalWrite(ETHERNET_PIN, HIGH);
  
  // Iniciamos la SD, y en caso que esta no inicie se retorna 1
  if(!Sd.begin(SD_PIN, SPI_FULL_SPEED)){
    digitalWrite(SD_PIN, HIGH);
    digitalWrite(ETHERNET_PIN, HIGH);
    return true;
  }

  if(!Sd.chdir(F("/registers"))){
    digitalWrite(SD_PIN, HIGH);
    digitalWrite(ETHERNET_PIN, HIGH);
    return true;
  }
  
  if(!MyFile.open("registers.csv", O_READ)){
    digitalWrite(SD_PIN, HIGH);
    digitalWrite(ETHERNET_PIN, HIGH);
    return true;
  }
  
  digitalWrite(SD_PIN, HIGH);
  digitalWrite(ETHERNET_PIN, HIGH);
  return false;
}


bool revisar_estado_sd(void){
  digitalWrite(SD_PIN, LOW);
  digitalWrite(ETHERNET_PIN, HIGH);
  
  // Iniciamos la SD, y en caso que esta no inicie se retorna 1
  if(!Sd.begin(SD_PIN, SPI_FULL_SPEED)){
    digitalWrite(SD_PIN, HIGH);
    digitalWrite(ETHERNET_PIN, HIGH);
    return true;
  }

  digitalWrite(SD_PIN, HIGH);
  digitalWrite(ETHERNET_PIN, HIGH);
  
  return false;
}


//////////////////////////////////////////////////////////////////
///         - FUNCIONES DE CONTROL PARA MODBUS -            //////
//////////////////////////////////////////////////////////////////


/*
 * Funcion que mapea el valor de las variables a los registros Modbus
 */
void modbus_var2reg(void) {
  int reg;  // Variable para indicar el registro a escribir

  // Registros 1 a 5: Variables generales
  reg = 0;
  modbus.bufferArray[0]   = idArduino;              // Reg 1
  modbus.bufferArray[1]   = estado_general;         // Reg 2
  modbus.bufferArray[2]   = estado_rtc;             // Reg 3
  modbus.bufferArray[3]   = estado_sd;              // Reg 4
  modbus.bufferArray[4]   = estado_modbus;          // Reg 5
  modbus.writeReg(reg, 5);

  // Registro 6 a 10: libre

  // Registro 11-12: Variables generales
  reg = 10;
  modbus.bufferArray[0]   = rtc_Ts;                // Reg 11
  modbus.bufferArray[1]   = lecturaAutomatica;    // Reg 12
  modbus.writeReg(reg, 2);

  // Registro 13-15: libre

  // Registro 16-33: Estado alarma
  reg = 15;
  modbus.bufferArray[0]   = estado_alarma_fuerte;    // Reg 16
  modbus.bufferArray[1]   = estado_alarma_suave;     // Reg 17
  for (register int i=0; i<NUMERO_CABLES; i++) modbus.bufferArray[2 +i] = estado_alarma_fuerte_cable[i];
  for (register int i=0; i<NUMERO_CABLES; i++) modbus.bufferArray[7 +i] = estado_alarma_suave_cable[i];
  for (register int i=0; i<NUMERO_CABLES; i++) modbus.bufferArray[12+i] = estado_alarma_cable[i];
  modbus.bufferArray[17]  = modo_alarma;             // Reg 33
  modbus.writeReg(reg, 18);

  // Registro 33-40: libre
  
  // Registro 41-46: fecha y hora del sistema
  reg = 40;
  DateTime fecha = RTC.now();
  modbus.bufferArray[0] = (uint16_t) fecha.year();
  modbus.bufferArray[1] = (uint16_t) fecha.month();
  modbus.bufferArray[2] = (uint16_t) fecha.day();
  modbus.bufferArray[3] = (uint16_t) fecha.hour();
  modbus.bufferArray[4] = (uint16_t) fecha.minute();
  modbus.bufferArray[5] = (uint16_t) fecha.second();
  modbus.writeReg(reg, 6);

  // Registro 47-52: fecha y hora para configuracion (solo de escritura por lo que aqui no se agrega)

  // Registro 53 a 60: libre

  // Registro 61 a 70: Largos externos
  reg = 60;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = largoExterno[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 71 a 80: Largo Interno
  reg = 70;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = largoInterno[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 81 a 90: Largo Nominal
  reg = 80;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = largoNominal[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);
  
  // Registro 91 a 100: Largo Medido
  reg = 90;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = largoMedido[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 101 a 110: Umbral alarma fuerte
  reg = 100;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = umbral_alarma_fuerte[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 111 a 120: Umbral alarma suave
  reg = 110;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = umbral_alarma_suave[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 121 a 130: Factor Velocidad
  reg = 120;
  for (int i=0; i<NUMERO_CABLES; i++){
    f2b.f = factorVelocidad[i];
    modbus.bufferArray[2*i    ] = f2b.b[1];
    modbus.bufferArray[2*i + 1] = f2b.b[0];
  }
  modbus.writeReg(reg, 2 * NUMERO_CABLES);

  // Registro 131 a 135: idx
  reg = 130;
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[i] = idx[i];
  modbus.writeReg(reg, NUMERO_CABLES);

  // Registro 136 a 140: Threshold 1
  reg = 135;
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[i] = th1[i];
  modbus.writeReg(reg, NUMERO_CABLES);

  // Registro 141 a 145: Threshold 2
  reg = 140;
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[i] = th2[i];
  modbus.writeReg(reg, NUMERO_CABLES);

  // Registro 146 a 150: Threshold 3
  reg = 145;
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[i] = th3[i];
  modbus.writeReg(reg, NUMERO_CABLES);

  // Registro 151 a 155: Datos nuevos por cable
  reg = 150;
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[i] = datosNuevoCable[i];
  modbus.writeReg(reg, NUMERO_CABLES);

  // Registros 156-199: Libres
  
  // Registro 200 a 212: Request de Modbus
  reg = 199;
  modbus.bufferArray[0]  = requestModbusLeerCables;           // Reg 200
  for (int i=0; i< NUMERO_CABLES; i++) modbus.bufferArray[1+i] = requestModbusLeerCable[i]; // Reg 201-205
  modbus.bufferArray[6]  = requestModbusReset;                // Reg 206
  modbus.bufferArray[7]  = requestModbusGuardarSD;            // Reg 207
  modbus.bufferArray[8]  = requestModbusLecturaAutomaticaON;  // Reg 208
  modbus.bufferArray[9]  = requestModbusLecturaAutomaticaOFF; // Reg 209
  modbus.bufferArray[10] = requestModbusAlarmaON;             // Reg 210
  modbus.bufferArray[11] = requestModbusAlarmaOFF;            // Reg 211
  modbus.bufferArray[12] = requestModbusAlarmaNormal;         // Reg 212
  modbus.bufferArray[13] = requestModbusAlarmaSilenciosa;     // Reg 213
  modbus.bufferArray[14] = requestModbusAlarmaDesactivada;    // Reg 214
  modbus.bufferArray[15] = requestModbusActualizarHora;       // Reg 215
  modbus.writeReg(reg, 16);
}


/*
 * Funcion que mapea el los registros Modbus a las variables.
 * Para que no se puedan cambiar los datos desde Modbus se deben comentar
 */
void modbus_reg2var(void) {
  int reg;  // Variable para indicar el registro a escribir
  
  // registro 11 y 12: Variables generales
  reg = 10;
  modbus.readReg(reg, 1);
  rtc_Ts            = modbus.bufferArray[0];
  // lecturaAutomatica = modbus.bufferArray[1];

  // Registro de 47 a 52: Variables de fecha y hora
  reg = 46;
  modbus.readReg(reg, 6);
  datetimeModbus = DateTime(
    (uint16_t) modbus.bufferArray[0], 
    (uint8_t)  modbus.bufferArray[1], 
    (uint8_t)  modbus.bufferArray[2],
    (uint8_t)  modbus.bufferArray[3],
    (uint8_t)  modbus.bufferArray[4],
    (uint8_t)  modbus.bufferArray[5]);

  // Registro 61 a 70: Largo externo
  reg = 60;
  modbus.readReg(reg, 10);
  for (int i=0; i< NUMERO_CABLES; i++){
    f2b.b[1] = modbus.bufferArray[2*i  ];
    f2b.b[0] = modbus.bufferArray[2*i+1];
    largoExterno[i] = f2b.f;
  }
  
  // Registro 71 a 80: Largo interno
  reg = 70;
  modbus.readReg(reg, 10);
  for (int i=0; i< NUMERO_CABLES; i++){
    f2b.b[1] = modbus.bufferArray[2*i  ];
    f2b.b[0] = modbus.bufferArray[2*i+1];
    largoInterno[i] = f2b.f;
  }

  // Registro 101 a 110: umbral fuerte de alarma
  reg = 100;
  modbus.readReg(reg, 10);
  for (int i=0; i< NUMERO_CABLES; i++){
    f2b.b[1] = modbus.bufferArray[2*i  ];
    f2b.b[0] = modbus.bufferArray[2*i+1];
    umbral_alarma_fuerte[i] = f2b.f;
  }

  // Registro 111 a 120 : umbral suave de alarma
  reg = 110;
  modbus.readReg(reg, 10);
  for (int i=0; i< NUMERO_CABLES; i++){
    f2b.b[1] = modbus.bufferArray[2*i  ];
    f2b.b[0] = modbus.bufferArray[2*i+1];
    umbral_alarma_suave[i] = f2b.f;
  }

  // Registro 121 a 130 : Factor de velocidad 
  reg = 120;
  modbus.readReg(reg, 10);
  for (int i=0; i< NUMERO_CABLES; i++){
    f2b.b[1] = modbus.bufferArray[2*i  ];
    f2b.b[0] = modbus.bufferArray[2*i+1];
    factorVelocidad[i] = f2b.f;
  }

  // Registro 131 a 135: idx
  reg = 130;
  modbus.readReg(reg, 5);
  for (int i=0; i< NUMERO_CABLES; i++) idx[i] = modbus.bufferArray[i];

  // Registro 136 a 140: th1
  reg = 135;
  modbus.readReg(reg, 5);
  for (int i=0; i< NUMERO_CABLES; i++) th1[i] = modbus.bufferArray[i];

  // Registro 141 a 145: th2
  reg = 140;
  modbus.readReg(reg, 5);
  for (int i=0; i< NUMERO_CABLES; i++) th2[i] = modbus.bufferArray[i];

  // Registro 146 a 150: th3
  reg = 145;
  modbus.readReg(reg, 5);
  for (int i=0; i< NUMERO_CABLES; i++) th3[i] = modbus.bufferArray[i];

  // Registro 151 a 155: Datos nuevos por cable
  reg = 150;
  modbus.readReg(reg, 5);
  for (int i=0; i< NUMERO_CABLES; i++) datosNuevoCable[i] = modbus.bufferArray[i];
  
  // Registro 200 a 212
  reg = 199;
  modbus.readReg(reg, 16);
  requestModbusLeerCables           = modbus.bufferArray[0];   // Reg 200
  requestModbusLeerCable[0]         = modbus.bufferArray[1];   // Reg 201
  requestModbusReset                = modbus.bufferArray[6];   // Reg 206
  requestModbusGuardarSD            = modbus.bufferArray[7];   // Reg 207
  requestModbusLecturaAutomaticaON  = modbus.bufferArray[8];   // Reg 208
  requestModbusLecturaAutomaticaOFF = modbus.bufferArray[9];   // Reg 209
  requestModbusAlarmaON             = modbus.bufferArray[10];  // Reg 210
  requestModbusAlarmaOFF            = modbus.bufferArray[11];  // Reg 211
  requestModbusAlarmaNormal         = modbus.bufferArray[12];  // Reg 212
  requestModbusAlarmaSilenciosa     = modbus.bufferArray[13];  // Reg 213
  requestModbusAlarmaDesactivada    = modbus.bufferArray[14];  // Reg 214
  requestModbusActualizarHora       = modbus.bufferArray[15];  // Reg 215
}


//////////////////////////////////////////////////////////////////
///           - FUNCIONES DE MANEJO DE REQUEST -            //////
//////////////////////////////////////////////////////////////////


/*
 * Funcion que lee los request del serial 0 (activa los flags)
 */
void Rx_request_serial0(void) {
  // Si no hay informacion en el buffer del serial, se termina la funcion
  if(!Serial.available()) return;

  // En caso contrario, leemos el string de entrada (char array de tamano 20)
  char str[30];
  for(int i=0; i<29; i++) {
    while(!Serial.available()){;}
    str[i] = Serial.read();
    if (str[i] == '\n'){
      str[i] = '\0';
      break;
    }
  }

  // Segun lo leido activamos, activamos los distintos request
  if      (strcmp(str, "RTC")          == 0) requestRTCLeerCables               = 1;
  
  else if (strcmp(str, "leerCables")   == 0) requestSerial0LeerCables           = 1;
  else if (strcmp(str, "exportar")     == 0) requestSerial0exportData           = 1;
  else if (strcmp(str, "cable1")       == 0) requestSerial0LeerCable[0]         = 1;
  else if (strcmp(str, "reset")        == 0) requestSerial0Reset                = 1;
  else if (strcmp(str, "guardarSD")    == 0) requestSerial0GuardarSD            = 1;
  else if (strcmp(str, "lectAutoON")   == 0) requestSerial0LecturaAutomaticaON  = 1;
  else if (strcmp(str, "lectAutoOFF")  == 0) requestSerial0LecturaAutomaticaOFF = 1;
  else if (strcmp(str, "alarmaON")     == 0) requestSerial0AlarmaON             = 1;
  else if (strcmp(str, "alarmaOFF")    == 0) requestSerial0AlarmaOFF            = 1;
  else if (strcmp(str, "alarmaNormal") == 0) requestSerial0AlarmaNormal         = 1;
  else if (strcmp(str, "alarmaSil")    == 0) requestSerial0AlarmaSilenciosa     = 1;
  else if (strcmp(str, "alarmaDesact") == 0) requestSerial0AlarmaDesactivada    = 1;
  else if (str[0] == 'd' && str[1] == 't' && str[2] == '=') {
    Serial.println(F(" Request configurar fecha y hora"));
  }
  else{
    #ifdef SERIAL0
      Serial.print(F("Request serial 0 no identificado: '")); Serial.print(str); Serial.println(F("'"));
      
    #endif
  }
}


/*
 * Funcion que lee los request de los perifericos (botones)
 */
void Rx_request_perifericos(void) {
  // Si esta definido el macro ALARMA_BOTON_ON, se lee el pin
  #ifdef ALARMA_BOTON_ON
    requestBotonON = !digitalRead(PIN_ALARMA_BOTON_ON);
  #endif

  // Si esta definido el macro ALARMA_BOTON_OFF, se lee el pin
  #ifdef ALARMA_BOTON_OFF
    requestBotonOFF = !digitalRead(PIN_ALARMA_BOTON_OFF);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLE_1, se lee el pin
  #ifdef PIN_BOTON_CABLE_1  
    requestBotonCable1 = !digitalRead(PIN_BOTON_CABLE_1);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLE_2, se lee el pin
  #ifdef PIN_BOTON_CABLE_2  
    requestBotonCable2 = !digitalRead(PIN_BOTON_CABLE_2);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLE_3, se lee el pin
  #ifdef PIN_BOTON_CABLE_3  
    requestBotonCable3 = !digitalRead(PIN_BOTON_CABLE_3);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLE_4, se lee el pin
  #ifdef PIN_BOTON_CABLE_4  
    requestBotonCable4 = !digitalRead(PIN_BOTON_CABLE_4);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLE_5, se lee el pin
  #ifdef PIN_BOTON_CABLE_5  
    requestBotonCable5 = !digitalRead(PIN_BOTON_CABLE_5);
  #endif

  // Si esta definido el macro PIN_BOTON_CABLES, se lee el pin
  #ifdef PIN_BOTON_CABLES 
    requestBotonCables = !digitalRead(PIN_BOTON_CABLES);
  #endif
}


/*
 * Funcion que revisa la hora del RTC (levante el flag)
 */
void Rx_request_RTC(void) {
  // si la lectura automatica no esta activa no se realiza nada
  if (!lecturaAutomatica) return;

  // En caso contrario se verifica si es necesario realizar la lectura
  DateTime fecha = RTC.now();                     // Leemos la hora del RTC
  uint32_t tiempo_actual = fecha.secondstime();   // Extraemos el tiempo actual (en segundos)
  // Verificamos si se debe realizar una lectura
  if(ultima_lectura + rtc_Ts < tiempo_actual) { 
    // Si se debe realizar una lectura, se levanta el request y se actualiza la hora
    requestRTCLeerCables = 1;
    ultima_lectura = tiempo_actual;
  }
}


/*
 * Funcion que resuelve los request del sistema. El sistema resulve un request y finaliza la funcion
 */
void resolver_request(void) {
  // Revisamos los request
  if      (requestRTCLeerCables) {
    #ifdef SERIAL0
      Serial.println(F("- Request RTC: Leer todos los cables"));
    #endif
    requestRTCLeerCables = 0;
    // Activamos la lectura de los cables
    for(int i=0; i<NUMERO_CABLES; i++) requestLeerCable[i] = 1;
    return;
  }
  else if (requestModbusLeerCables) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: Leer todos los cables"));
    #endif
    requestModbusLeerCables = 0;
    // Activamos la lectura de los cables
    for(int i=0; i<NUMERO_CABLES; i++) requestLeerCable[i] = 1;
    return;
  }
  else if (requestModbusLecturaAutomaticaON) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: lectura automatica ON"));
    #endif
    requestModbusLecturaAutomaticaON = 0;
    lecturaAutomatica = true;
    return;
  }
  else if (requestModbusLecturaAutomaticaOFF) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: lectura automatica OFF"));
    #endif
    requestModbusLecturaAutomaticaOFF = 0;
    lecturaAutomatica = false;
    return;
  }
  else if (requestModbusAlarmaON) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: alarma ON"));
    #endif
    requestModbusAlarmaON = 0;
    for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 2;
    alarma_ON();
    return;
  }
  else if (requestModbusAlarmaOFF) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: alarma OFF"));
    #endif
    requestModbusAlarmaOFF = 0;
    for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 0;
    alarma_OFF();
    return;
  }
  else if (requestModbusAlarmaNormal) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: alarma normal"));
    #endif
    requestModbusAlarmaNormal = 0;
    modo_alarma = 1;
    return;
  }
  else if (requestModbusAlarmaSilenciosa) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: alarma silenciosa"));
    #endif
    requestModbusAlarmaSilenciosa = 0;
    modo_alarma = 2;
    return;
  }
  else if (requestModbusAlarmaDesactivada) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: alarma desactivada"));
    #endif
    requestModbusAlarmaDesactivada = 0;
    modo_alarma = 0;
    return;
  }
  else if (requestModbusGuardarSD) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: Guardar en SD"));
    #endif
    requestModbusGuardarSD = 0;
    actualizar_Config_SD();
    return;
  }
  else if (requestModbusActualizarHora) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: Actualizar hora"));
    #endif
    requestModbusActualizarHora = 0;
    actualizar_hora();
    return;
  }
  else if (requestModbusReset) {
    #ifdef SERIAL0
      Serial.println(F("- Request Modbus: reiniciar el sistema"));
    #endif
    requestModbusReset = 0;
    wdt_enable(WDTO_15MS);
    for (;;) {}
    return;
  }
  else if (requestSerial0LeerCables) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: Leer todos los cables"));
    #endif
    requestSerial0LeerCables = 0;
    // Activamos la lectura de los cables
    for(int i=0; i<NUMERO_CABLES; i++) requestLeerCable[i] = 1;
    return;
  }
  else if (requestSerial0exportData) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: Exportar datos"));
    #endif
    requestSerial0exportData = 0;
    exportarDatosSerial();
    return;
  }
  else if (requestSerial0LecturaAutomaticaON) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: lectura automatica ON"));
    #endif
    requestSerial0LecturaAutomaticaON = 0;
    lecturaAutomatica = true;
    return;
  }
  else if (requestSerial0LecturaAutomaticaOFF) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: lectura automatica OFF"));
    #endif
    requestSerial0LecturaAutomaticaOFF = 0;
    lecturaAutomatica = false;
    return;
  }
  else if (requestSerial0AlarmaON) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: alarma ON"));
    #endif
    requestSerial0AlarmaON = 0;
    for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 2;
    alarma_ON();
    return;
  }
  else if (requestSerial0AlarmaOFF) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: alarma OFF"));
    #endif
    requestSerial0AlarmaOFF = 0;
    for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 0;
    alarma_OFF();
    return;
  }
  else if (requestSerial0AlarmaNormal) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: alarma normal"));
    #endif
    requestSerial0AlarmaNormal = 0;
    modo_alarma = 1;
    return;
  }
  else if (requestSerial0AlarmaSilenciosa) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: alarma silenciosa"));
    #endif
    requestSerial0AlarmaSilenciosa = 0;
    modo_alarma = 2;
    return;
  }
  else if (requestSerial0AlarmaDesactivada) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: alarma desactivada"));
    #endif
    requestSerial0AlarmaDesactivada = 0;
    modo_alarma = 0;
    return;
  }
  else if (requestSerial0GuardarSD) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: Guardar en SD"));
    #endif
    requestSerial0GuardarSD = 0;
    actualizar_Config_SD();
    return;
  }
  else if (requestSerial0Reset) {
    #ifdef SERIAL0
      Serial.println(F("- Request Serial 0: Reset"));
    #endif
    requestSerial0Reset = 0;
    wdt_enable(WDTO_15MS);
    for (;;) {}
    return;
  }

  #ifdef ALARMA_BOTON_ON
    else if (requestBotonON) {
      #ifdef SERIAL0
        Serial.println(F("- Request boton: Alarma ON"));
      #endif
      requestBotonON = 0;
      for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 2;
      alarma_ON();
      return;
    }
  #endif

  #ifdef ALARMA_BOTON_OFF
    else if (requestBotonOFF) {
      #ifdef SERIAL0
        Serial.println(F("- Request boton: Alarma OFF"));
      #endif
      requestBotonOFF = 0;
      for(register int i=0; i<NUMERO_CABLES; i++) estado_alarma_cable[i] = 0;
      alarma_OFF();
      return;
    }
  #endif

  #ifdef PIN_BOTON_CABLE_1
    else if (requestBotonCable1) {
      #ifdef SERIAL0
        Serial.println(F("- Request boton: Leer cable 1"));
      #endif
      requestBotonCable1 = 0;
      requestLeerCable[0] = 1;
      return;
    }
  #endif

  #ifdef PIN_BOTON_CABLES
    else if (requestBotonCables) {
      #ifdef SERIAL0
        Serial.println(F("- Request boton: Leer todos los cables"));
      #endif
      requestBotonCables = 0;
      // Activamos la lectura de los cables
      for(int i=0; i<NUMERO_CABLES; i++) requestLeerCable[i] = 1;
      return;
    }
  #endif
  
  for (int i=0; i<NUMERO_CABLES; i++){
    if (requestModbusLeerCable[i]){
      #ifdef SERIAL0
        Serial.print(F("- Request Modbus: Leer cable "));  Serial.println(i + 1);
      #endif
      requestModbusLeerCable[i] = 0;
      requestLeerCable[i] = 1;
      return;
    }
    else if(requestSerial0LeerCable[i]){
      #ifdef SERIAL0
        Serial.print(F("- Request Serial 0: Leer cable ")); Serial.println(i + 1);
      #endif
      requestSerial0LeerCable[i] = 0;
      requestLeerCable[i] = 1;
      return;
    }
    else if (requestLeerCable[i]){
      leerCable(i + 1);
      requestLeerCable[i] = 0;
      return;
    }
  }
}

void exportarDatosSerial(void){
  // Activamos la lectura de los cables
  dataLog.changeFolderPath("");
  dataLog.changeFileName("datos.csv");
  
  int sts = dataLog.exportDataSerial();
  Serial.print("Estado de dato exportado: ");Serial.println(sts);
  switch(sts){
    case 0:
      #ifdef SERIAL0
        Serial.println(F(" -> Datos correctamente exportados"));
      #endif
      sts = dataLog.removeFile();
      if (sts == 0){
        #ifdef SERIAL0
          Serial.println(F(" -> buffer eliminado"));
        #endif
      }
      else{
        #ifdef SERIAL0
          Serial.println(F(" -> buffer no se pudo eliminado"));
        #endif
      }
      break;

    case 4:
      #ifdef SERIAL0
          Serial.println(F(" -> Archivo para exportar no existente"));
        #endif
      break;
    
    case 5:
      #ifdef SERIAL0
        Serial.println(F(" -> Datos recibidos no coinciden con los leidos en el archivo"));
      #endif
      break;

    case 7:
      #ifdef SERIAL0
        Serial.println(F(" -> Tiempo de espera de exportacion de archivos maximo excedido"));
      #endif
      break;
      
    default:
      #ifdef SERIAL0
        Serial.println(F(" -> Datos no exportados. Ocurrio un problema"));
      #endif
      break;
  }
}

//////////////////////////////////////////////////////////////////
///           - FUNCIONES DE LECTURA DE CABLES -            //////
//////////////////////////////////////////////////////////////////

/*
 * Funcion para leer un cable
 */
void leerCable(uint8_t Ncable) {
  // Extraemos y calculamos las variables necesarias para la lectura
  if (Ncable == 0 || Ncable > NUMERO_CABLES) return;
  digitalWrite(PIN_LECTURA, HIGH);
  uint16_t largo      = (uint16_t) largoNominal[Ncable-1];  // Largo del cable (redondeado a un entero de 2 bytes)
  uint16_t N_muestras = definirMuestras(largo);             // Calculamos el numero de muestras
  
  #ifdef SERIAL0
    Serial.println(F("------------------------------------"));
    Serial.print  (F("-           Leer Cable ")); Serial.print(Ncable); 
    Serial.println(F("           -"));
    Serial.println(F("------------------------------------"));
    Serial.print(F("-> Largo cable      : ")); Serial.println(largoNominal[Ncable-1]);
    Serial.print(F("-> Total de muestras: ")); Serial.println(N_muestras);
  #endif
  
  for(int j=0; j<MF_WINDOWS_LENGTH; j++){
    // Reiniciamos la Arty
    digitalWrite(A15, LOW);
    delay(200);
    digitalWrite(A15, HIGH);
    delay(200);
    // Generamos un mensaje para enviar a la ARTY
    uint8_t ARTY_msj[5];
    ARTY_msj[0] = (uint8_t) BYTE_INSTRUCCION_LECTURA;
    ARTY_msj[1] = Ncable; 
    ARTY_msj[2] = (uint8_t) ((N_muestras - 1) >> 8) & 0xFF;
    ARTY_msj[3] = (uint8_t) ((N_muestras - 1) % 0xFF);
    ARTY_msj[4] = largo;
  
    // Enviamos el mensaje
    Serial1.write(ARTY_msj, 5);
    Serial1.flush();
    #ifdef SERIAL0
      Serial.println(F("-> Instruccion enviada a ARTY"));
      Serial.println(F("-> Esperando respuesta de ARTY . . ."));
    #endif
    
    // Recibiendo datos
    if(!recibirDatosSerial1()){
      #ifdef SERIAL0
        Serial.println(F("-> Error al recibir los datos: Tiempo de respuesta superado"));
        largoMedido[Ncable - 1] = -1.0;
      #endif
      digitalWrite(PIN_LECTURA, LOW);
      return;
    }
    
    #ifdef SERIAL0
      Serial.println(F("-> Respuesta recibida!"));
      Serial.println(F("-> Datos recibidos:"));
      for (uint16_t i=0; i<N_muestras; i++){
        Serial.print(F(" - Dato ")); Serial.print(i+1); 
        Serial.print(F(": ")); Serial.println(datosCable[i]);
      }
    #endif
  
    // Aplicamos el filtro
    #ifdef SERIAL0
      Serial.println(F("-> agregando datos al buffer . . ."));
    #endif
    mFilter[Ncable - 1].addData(datosCable, N_muestras);

    // Guardando los datos en SD
    #ifdef SERIAL0
      Serial.println(F(" -> Datos escritos en Modbus"));
      Serial.println(F(" -> Guardando datos en tarjeta SD . . ."));
    #endif
    guardarDatosSD(Ncable, N_muestras);
  }

  // Escribimos los datos en Modbus
  #ifdef SERIAL0
    Serial.println(F(" -> Escribiendo datos en Modbus . . ."));
  #endif
  guardarDatosModbus(Ncable, MAXMUESTRAS);
  
  // Rutina luego de realizar las N mediciones seguidas
  #ifdef SERIAL0
    Serial.println(F("-> Filtrando datos . . ."));
  #endif
  // Filtramos los datos
  mFilter[Ncable - 1].getFilterData(datosCable, N_muestras);
  #ifdef SERIAL0
    Serial.println(F("-> Datos filtrados: "));
    for (uint16_t i=0; i<N_muestras; i++){
      Serial.print(F(" - Dato ")); Serial.print(i+1); 
      Serial.print(F(": ")); Serial.println(datosCable[i]);
    }
  #endif

  largoMedido[Ncable - 1] = calcularLargoCable(Ncable, N_muestras);
  #ifdef SERIAL0
    Serial.print(F(" -> Largo Medido: ")); Serial.println(largoMedido[Ncable - 1]);
  #endif

   // Actualizamos los datos por modbus
  #ifdef SERIAL0
    Serial.println(F(" -> Escribiendo datos filtrados en Modbus . . ."));
  #endif
  guardarDatosFiltradosModbus(Ncable, MAXMUESTRAS);

  #ifdef SERIAL0
    Serial.println(F(" -> Datos escritos en Modbus"));
    Serial.println(F(" -> Revisando Alarma"));
  #endif

  revisarAlarma(Ncable);

  #ifdef SERIAL0
    Serial.println(F(" -> Guardando datos filtrados en tarjeta SD . . ."));
  #endif
 
  guardarDatosFiltradosSD(Ncable, N_muestras);

  guardarDatosArchivo(Ncable, N_muestras);
  #ifdef SERIAL0
    Serial.println(F(" -> Datos guardados en tarjeta SD"));
  #endif
  
  // Actualizamos la ultima lectura
  DateTime fecha = RTC.now();             // Leemos la hora del RTC
  ultima_lectura = fecha.secondstime();   // Extraemos el tiempo actual

  // Levantamos el flag de que se realizo una nueva lectura
  datosNuevoCable[Ncable - 1] = 1;
  digitalWrite(PIN_LECTURA, LOW);
}


/*
 * Funcion que calcula el largo de un cable
 */
float calcularLargoCable(uint8_t Ncable, uint16_t N_muestras){
  int inde          = (int) idx[Ncable-1];
  uint16_t threshold_1   = th1[Ncable-1];
  uint16_t threshold_2   = th2[Ncable-1];
  uint16_t threshold_3   = th3[Ncable-1];
  float factor_velocidad = factorVelocidad[Ncable-1];
  float largoMedido;
  int dt;
  int index1 = 0;
  int index2 = 0;
  bool cond1, cond2, cond3;
  for(uint16_t i=inde; i<N_muestras; i++) {
    if (index1 == 0){
      cond1 = (int) datosCable[i] - (int) datosCable[i-1] > (int) threshold_1; // Derivada mayor que un umbral
      cond2 = datosCable[i] > threshold_2;                                     // Valor absoluto mayor que otro umbral
      if (cond1 & cond2) index1 = i;
    }
    else {
      cond3 = datosCable[i] > threshold_3;
      if (cond3){
        index2 = i;
        break;
      }
    }
  }
  
  dt = index2 - index1;
  if (dt < 0) return -1.0;
  
  largoMedido = factor_velocidad * (float) dt;
  return largoMedido;
}


/*
 * Funcion que retorna el largo de un cable en funcion del largo del cable
 */
uint16_t definirMuestras(uint16_t largo) {
  if (2 * largo + 50 < MAXMUESTRAS) {
    return (2 * largo + 50); // aqui se puede elegir otra cantidad de muestras extra, para todo el sketch
  }
  else {
    return MAXMUESTRAS; //no queremos pasarnos de los datos maximos definidos, y tener otros problemas despues...
  }
}


/*
 * Funcion para recibir los datos desde la ARTY
 */
bool recibirDatosSerial1(void){
  uint8_t buff; // Buffer para recibir el dato
  uint16_t i = 0;    // Contador

  // leemos la hora y la guardamos en una variable
  DateTime fecha = RTC.now();                   // Leemos la hora del RTC
  uint32_t tiempo_inicial = fecha.secondstime();  // Extraemos la hora en segundos
  
  while(true){
    // Esperamos que exista informacion en el serial
    if(Serial1.available()){
      buff = Serial1.read();  // Leemos un byte

      // Dependiendo del valor que tenga el buffer, se toman las acciones
      if     (buff == BYTE_INICIO_RECIBIR) {;}    // Byte de inicio. No se hace nada
      else if(buff == BYTE_FIN_RECIBIR){break;}   // Byte final. Se corta el bucle
      else{                                       // El byte recibido y el siguiente son un entero de 2 bytes
        datosCable[i] = (buff) * 256;     // Sumamos el valor desplazado en 8 bits
        while(!Serial1.available()) {;}   // Esperamos a que haya informacion en el serial
        datosCable[i] += Serial1.read();  // Leemos el dato y lo sumamos al byte anterior
        i++;                              // Aumentamos el contador
      }
    }
    fecha = RTC.now();
    uint32_t tiempo_actual = fecha.secondstime();
    if (tiempo_inicial + DEAD_TIME_ARTY < tiempo_actual) return false;
  }
  while (i < MAXMUESTRAS){
        datosCable[i] = 0;
        i++;
      }
  return true;
}


/*
 * Funcion que actualiza los registros de los datos a modbus
 */
void guardarDatosModbus(uint8_t Ncable, uint16_t N_muestras){
  int bufferSize = 100;                         // Tamano del buffer para escribir registros en modbus
  uint16_t reg   = registrosCables[Ncable - 1]; // Registro inicial de Modbus
  int N = N_muestras / bufferSize;              // Numero de veces a iterar para escribir BufferSize registros
  int reg_ini, reg_fin;
  
  for(int n=0; n<N; n++) {
    // Calculamos los registros que se escribiran
    reg_ini = reg + n * bufferSize;
    reg_fin = reg + (n + 1) * bufferSize;
    #ifdef SERIAL0
      Serial.print(F(" - Registros escritos: ")); Serial.print(reg_ini + 1);
      Serial.print(F(" - ")); Serial.println(reg_fin);
    #endif 

    // Actualizamos el buffer de modbus
    for(int i=0; i<bufferSize; i++) modbus.bufferArray[i] = datosCable[n*bufferSize + i];

    // Escribimos los registros Modbus
    modbus.writeReg(reg_ini, bufferSize);
  }

  // En caso de quede un ultimo segmento sin escribir, este se escribe
  int resto = N_muestras % bufferSize;
  if (resto) {
    // Calculamos los registros que se escribiran
    reg_ini = reg + N * bufferSize;
    reg_fin = reg + N * bufferSize + resto;
    #ifdef SERIAL0
      Serial.print(F(" - Registros escritos: ")); Serial.print(reg_ini + 1);
      Serial.print(F(" - ")); Serial.println(reg_fin);
    #endif 

    // Actualizamos el buffer de modbus
    for(int i=0; i<resto; i++) modbus.bufferArray[i] = datosCable[N*bufferSize + i];

    // Escribimos los registros Modbus
    modbus.writeReg(reg_ini, resto);
  }
}


void guardarDatosFiltradosModbus(uint8_t Ncable, uint16_t N_muestras){
  int bufferSize = 100;                         // Tamano del buffer para escribir registros en modbus
  uint16_t reg   = registrosCablesFiltroMediano[Ncable - 1]; // Registro inicial de Modbus
  int N = N_muestras / bufferSize;              // Numero de veces a iterar para escribir BufferSize registros
  int reg_ini, reg_fin;
  
  for(int n=0; n<N; n++) {
    // Calculamos los registros que se escribiran
    reg_ini = reg + n * bufferSize;
    reg_fin = reg + (n + 1) * bufferSize;
    #ifdef SERIAL0
      Serial.print(F(" - Registros escritos: ")); Serial.print(reg_ini + 1);
      Serial.print(F(" - ")); Serial.println(reg_fin);
    #endif 

    // Actualizamos el buffer de modbus
    for(int i=0; i<bufferSize; i++) modbus.bufferArray[i] = datosCable[n*bufferSize + i];

    // Escribimos los registros Modbus
    modbus.writeReg(reg_ini, bufferSize);
  }

  // En caso de quede un ultimo segmento sin escribir, este se escribe
  int resto = N_muestras % bufferSize;
  if (resto) {
    // Calculamos los registros que se escribiran
    reg_ini = reg + N * bufferSize;
    reg_fin = reg + N * bufferSize + resto;
    #ifdef SERIAL0
      Serial.print(F(" - Registros escritos: ")); Serial.print(reg_ini + 1);
      Serial.print(F(" - ")); Serial.println(reg_fin);
    #endif 

    // Actualizamos el buffer de modbus
    for(int i=0; i<resto; i++) modbus.bufferArray[i] = datosCable[N*bufferSize + i];

    // Escribimos los registros Modbus
    modbus.writeReg(reg_ini, resto);
  }
}


/*
 * Funcion para guardar los datos en la tarjeta SD
 */
void guardarDatosSD(uint8_t Ncable, uint16_t N_muestras){
  // Abrimos la carpeta  
  char file_path[8];
  sprintf(file_path, "Cable_%d", Ncable);
  dataLog.changeFolderPath(file_path);
  
  // Cambiamos el nombre del archivo archivo
  DateTime fecha = RTC.now();
  char file_name[15];
  sprintf(file_name, "%04d_%02d_%02d_raw.csv", fecha.year(), fecha.month(), fecha.day());
  dataLog.changeFileName(file_name);

  #ifdef SERIAL0
      Serial.print(F(" - Carpeta: ")); Serial.println(file_path);
      Serial.print(F(" - Archivo: ")); Serial.println(file_name);
    #endif 

  // Configuramos el header

  if (!dataLog.checkHeader()){
    dataLog.writeHeader("datetime; Arduino ID; N cable");
    char s[20];
    for(int i=0; i<MAXMUESTRAS; i++) {
      sprintf(s, "muestra %i", i+1);
      dataLog.writeHeader(s);
    }
    dataLog.newRow();
  }
  
  // Escribimos los metadatos
  // dataLog.datetime = fecha;
  char metaData[20];
  sprintf(metaData, "%04d-%02d-%02d %02d:%02d:%02d", fecha.year(), fecha.month(), fecha.day(), fecha.hour(), fecha.minute(), fecha.second());
  dataLog.writeChar(metaData);
  sprintf(metaData, "%d; %d", idArduino, Ncable);
  dataLog.writeChar(metaData);
  // Escribimos los datos
  dataLog.writeInt((int*) datosCable, N_muestras);

  // Escribimos una nueva linea
  dataLog.newRow();
}


void guardarDatosFiltradosSD(uint8_t Ncable, uint16_t N_muestras){
  // Abrimos la carpeta  
  char file_path[8];
  sprintf(file_path, "Cable_%d", Ncable);
  dataLog.changeFolderPath(file_path);
  
  // Cambiamos el nombre del archivo archivo
  DateTime fecha = RTC.now();
  char file_name[15];
  sprintf(file_name, "%04d_%02d_%02d.csv", fecha.year(), fecha.month(), fecha.day());
  dataLog.changeFileName(file_name);

  #ifdef SERIAL0
      Serial.print(F(" - Carpeta: ")); Serial.println(file_path);
      Serial.print(F(" - Archivo: ")); Serial.println(file_name);
    #endif 

  // Configuramos el header

  if (!dataLog.checkHeader()){
    dataLog.writeHeader("datetime; Arduino ID; N cable");
    dataLog.writeHeader("L nominal; L medido; estado alarma");
    dataLog.writeHeader("alarma fuerte; alarma suave");
    dataLog.writeHeader("L externo; L interno; umbral fuerte");
    dataLog.writeHeader("umbral suave; fact velocidad");
    dataLog.writeHeader("index; th1; th2; th3");
    char s[20];
    for(int i=0; i<MAXMUESTRAS; i++) {
      sprintf(s, "muestra %i", i+1);
      dataLog.writeHeader(s);
    }
    dataLog.newRow();
  }
  
  // Escribimos los metadatos
  char metaData[20];
  sprintf(metaData, "%04d-%02d-%02d %02d:%02d:%02d", fecha.year(), fecha.month(), fecha.day(), fecha.hour(), fecha.minute(), fecha.second());
  dataLog.writeChar(metaData);
  sprintf(metaData, "%d; %d", idArduino, Ncable);
  dataLog.writeChar(metaData);
  dataLog.writeFloat(largoNominal, 1);
  dataLog.writeFloat(largoMedido, 1);
  dataLog.writeInt((const int *) estado_alarma_cable, 1);
  sprintf(metaData, "%i; %i", (int) estado_alarma_fuerte_cable[0], (int) estado_alarma_suave_cable[0]);
  dataLog.writeChar(metaData);
  dataLog.writeFloat(largoExterno, 1);
  dataLog.writeFloat(largoInterno, 1);
  dataLog.writeFloat(umbral_alarma_fuerte, 1);
  dataLog.writeFloat(umbral_alarma_suave, 1);
  dataLog.writeFloat(factorVelocidad, 1);
  dataLog.writeInt((const int *) idx, 1);
  dataLog.writeInt((const int *) th1, 1);
  dataLog.writeInt((const int *) th2, 1);
  dataLog.writeInt((const int *) th3, 1);
  
  // Escribimos los datos
  dataLog.writeInt((int*) datosCable, N_muestras);

  // Escribimos una nueva linea
  dataLog.newRow();
}

void guardarDatosArchivo(uint8_t Ncable, uint16_t N_muestras){
  // Abrimos la carpeta  
  dataLog.changeFolderPath("");
  
  // Cambiamos el nombre del archivo archivo
  DateTime fecha = RTC.now();
  dataLog.changeFileName("datos.csv");

  // Configuramos el header

  if (!dataLog.checkHeader()){
    dataLog.writeHeader("datetime; Arduino ID; N cable");
    dataLog.writeHeader("L nominal; L medido; estado alarma");
    dataLog.writeHeader("alarma fuerte; alarma suave");
    dataLog.writeHeader("L externo; L interno; umbral fuerte");
    dataLog.writeHeader("umbral suave; fact velocidad");
    dataLog.writeHeader("index; th1; th2; th3");
    char s[20];
    for(int i=0; i<MAXMUESTRAS; i++) {
      sprintf(s, "muestra %i", i+1);
      dataLog.writeHeader(s);
    }
    dataLog.newRow();
  }
  
  // Escribimos los metadatos
  char metaData[20];
  sprintf(metaData, "%04d-%02d-%02d %02d:%02d:%02d", fecha.year(), fecha.month(), fecha.day(), fecha.hour(), fecha.minute(), fecha.second());
  dataLog.writeChar(metaData);
  sprintf(metaData, "%d; %d", idArduino, Ncable);
  dataLog.writeChar(metaData);
  dataLog.writeFloat(largoNominal, 1);
  dataLog.writeFloat(largoMedido, 1);
  dataLog.writeInt((const int *) estado_alarma_cable, 1);
  sprintf(metaData, "%i; %i", (int) estado_alarma_fuerte_cable[0], (int) estado_alarma_suave_cable[0]);
  dataLog.writeChar(metaData);
  dataLog.writeFloat(largoExterno, 1);
  dataLog.writeFloat(largoInterno, 1);
  dataLog.writeFloat(umbral_alarma_fuerte, 1);
  dataLog.writeFloat(umbral_alarma_suave, 1);
  dataLog.writeFloat(factorVelocidad, 1);
  dataLog.writeInt((const int *) idx, 1);
  dataLog.writeInt((const int *) th1, 1);
  dataLog.writeInt((const int *) th2, 1);
  dataLog.writeInt((const int *) th3, 1);
  
  // Escribimos los datos
  dataLog.writeInt((int*) datosCable, N_muestras);

  // Escribimos una nueva linea
  dataLog.newRow();
}

void actualizar_hora(void){
  #ifdef SERIAL0
    Serial.print(F("- Actualizando hora: "));
    Serial.print(datetimeModbus.year());  Serial.print("-");
    Serial.print(datetimeModbus.month());  Serial.print("-");
    Serial.print(datetimeModbus.day());  Serial.print(" ");
    Serial.print(datetimeModbus.hour());  Serial.print(":");
    Serial.print(datetimeModbus.minute());  Serial.print(":");
    Serial.print(datetimeModbus.second());  Serial.print('\n');
  #endif
  RTC.adjust(datetimeModbus);
  
}


//////////////////////////////////////////////////////////////////
///           - FUNCIONES DE ALARMA -                       //////
//////////////////////////////////////////////////////////////////

void statusAlarma(void){
  for(register int n=0; n<NUMERO_CABLES; n++){
    estado_alarma_fuerte_cable[n] = false;
    estado_alarma_suave_cable[n] = false;
    estado_alarma_fuerte = false;
    estado_alarma_suave = false;
    switch(modo_alarma){
        // Caso de alarma desactivada
        case 0:     
          break;
        // Caso de alarma normal
        case 1:
          if(estado_alarma_cable[n] == 2) {
            estado_alarma_fuerte_cable[n] = true;
            estado_alarma_fuerte = true;
          }
          if((estado_alarma_cable[n] >= 1) && (estado_alarma_cable[n] <= 3)) {
            estado_alarma_suave_cable[n] = true;
            estado_alarma_suave = true;
          }
          break;
        // Caso de alarma silenciosa
        case 2:
          if((estado_alarma_cable[n] >= 1) && (estado_alarma_cable[n] <= 3)) {
            estado_alarma_suave_cable[n] = true;
            estado_alarma_suave = true;
          }
    }
  }

  for(register int n=0; n<NUMERO_CABLES; n++) estado_alarma_fuerte |= estado_alarma_fuerte_cable[n];
  for(register int n=0; n<NUMERO_CABLES; n++) estado_alarma_suave |= estado_alarma_suave_cable[n];
  if(estado_alarma_fuerte) alarma_ON();
  else alarma_OFF();
}



/*
 * Funcion para revisar la alarma
 */
void revisarAlarma(uint8_t Ncable) {
  #ifdef ALARMA
    float l_medido  = largoMedido[Ncable - 1];
    float f_vel     = factorVelocidad[Ncable - 1];
    float l_externo = largoExterno[Ncable - 1];
    float l_nominal = largoNominal[Ncable - 1];
    float umbral_suave  = umbral_alarma_suave [Ncable - 1];
    float umbral_fuerte = umbral_alarma_fuerte[Ncable - 1];
  
    if      (l_medido <= 0)                         estado_alarma_cable[Ncable - 1] = 5;
    else if (l_medido <= 1.1*f_vel)                 estado_alarma_cable[Ncable - 1] = 4;
    else if (l_medido <= l_externo - 1.1*f_vel)     estado_alarma_cable[Ncable - 1] = 3;
    else if (l_medido <= l_nominal - umbral_suave)  estado_alarma_cable[Ncable - 1] = 2;
    else if (l_medido <= l_nominal - umbral_fuerte) estado_alarma_cable[Ncable - 1] = 1;
    else                                            estado_alarma_cable[Ncable - 1] = 0;
  
    #ifdef SERIAL0
      Serial.print(F(" - Estado de la alarma: "));
      switch(estado_alarma_cable[Ncable -1]){
        case 0:
          Serial.println(F("Desactivada. Medicion dentro de rango normal"));
          break;
        case 1:
          Serial.println(F("Software. Medicion dentro de rango preocupante"));
          break;
        case 2:
          Serial.println(F("software y hardware. Medicion dentro de rango peligroso"));
          break;
        case 3:
          Serial.println(F("software. Medicion entre TDR y largo externo"));
          break;
        case 4:
          Serial.println(F("Desactivada. Cable no conectado"));
          break;
        case 5:
          Serial.println(F("Desactivada. Medicion incorrecta"));
          break;
      }
    #endif
  #endif
}

/*
 * Funcion que activa la alarma
 */
void alarma_ON(void){
  // Definimos LOW en los pines (LOW activa los reles)
  digitalWrite(PIN_RELE_1, LOW);
  digitalWrite(PIN_RELE_2, LOW);
}


/*
 * Funcion que desactiva la alarma
 */
void alarma_OFF(void){
    digitalWrite(PIN_RELE_1, HIGH);
    digitalWrite(PIN_RELE_2, HIGH);
}


/*
 * Funcion que lee el archivo de configuracion
 */
void actualizar_Config_SD(void){
  // Activamos la SD y seteamos la SD al root
  digitalWrite(PIN_ETHERNET, HIGH);
  digitalWrite(PIN_SD, LOW);
  Sd.chdir();
  #ifdef SERIAL0
    Serial.print(F("- Abriendo archivo config . . . "));
  #endif
  
  // Creamos y abrimos el archivo para lectura
  if (!MyFile.open(CONFIG_FILE_NAME, O_READ)){
    digitalWrite(PIN_ERROR_SD, HIGH);
    #ifdef SERIAL0
      Serial.println(F(" No se pudo encontrar el archivo de configuracion"));
      Serial.println(F(""));
    #endif
    // Dado que no se pudo iniciar el archivo, se termina la funcion
    MyFile.close();
    return;
  }

  #ifdef SERIAL0
    Serial.println(F("Ok!"));
    Serial.print(F("- Abriendo buffer de config . . . "));
  #endif  
  
  // Abrimos un nuevo archivo para guardar los datos
  SdFile buffFile;      // Instancia para manipular datos
  // Creaos y abrimos el buffer
  if (!buffFile.open("bufferConfig.txt", O_WRITE | O_CREAT)){
    digitalWrite(PIN_ERROR_SD, HIGH);
    #ifdef SERIAL0
      Serial.println(F(" no se pudo crear el buffer para archivo de configuracion"));
      Serial.println(F(""));
    #endif
    MyFile.close();
    buffFile.close();
    return;
  }
  
  #ifdef SERIAL0
    Serial.println(F("Ok!"));
    Serial.println(F("- Guardando archivo de config . . ."));
  #endif
    // Creamos char array para leer los datos
  char buff1[100];          // Primer Array para leer los datos
  char buff2[100];          // Segundo Array para leer los datos
  char original_str[100];   // Char array para guardar el dato orginal
  char value[50];           // Array para guardar el valor de la variable

  char end_str[] = "\n";
  // Mientras hayan datos en el archivo
  while(MyFile.available()){
    // Leemos una linea del archivo en buff1 (lo reespaldamos en original_str
    MyFile.fgets(original_str, sizeof(buff1), end_str);
    memcpy(buff1, original_str, sizeof(buff1));

    remove_space(buff2, buff1);     // Removemos los espacios

    // Si el char array parte con // o NULL se ignora
    if ((buff1[0] != '/') && (buff1[1] != '/') && buff1[0] != '\0' && buff1[0] != '\n' && buff1[0] != '\r'){

      // Dividimos el array en el nombre de la variable (guardado en buff2) y en value
      split_array(buff2, value, buff1);
      // Variables principales
      if      (strcmp(buff2, "idAr")        == 0)  itoa(idArduino,   value, 10);
      else if (strcmp(buff2, "Ts")          == 0)  itoa(rtc_Ts,      value, 10);
      else if (strcmp(buff2, "modo_alarma") == 0)  itoa(modo_alarma, value, 10);

      // Configuracion del servidor
      else if (strcmp(buff2, "serv") == 0)  sprintf(value, "%i.%i.%i.%i", server[0], server[1], server[2], server[3]);
      else if (strcmp(buff2, "myip") == 0)  sprintf(value, "%i.%i.%i.%i", ip[0], ip[1], ip[2], ip[3]);
      else if (strcmp(buff2, "dnSe") == 0)  sprintf(value, "%i.%i.%i.%i", dnServer[0], dnServer[1], dnServer[2], dnServer[3]);
      else if (strcmp(buff2, "gate") == 0)  sprintf(value, "%i.%i.%i.%i", gateway[0], gateway[1], gateway[2], gateway[3]);
      else if (strcmp(buff2, "mask") == 0)  sprintf(value, "%i.%i.%i.%i", subnet[0], subnet[1], subnet[2], subnet[3]);
      else if (strcmp(buff2, "mac_") == 0)  sprintf(value, "%02X.%02X.%02X.%02X.%02X.%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      
      // Datos por cable
      else if (strcmp(buff2, "LargoExterno1") == 0)  dtostrf(largoExterno[0], 8, 8, value);

      else if (strcmp(buff2, "LargoInterno1") == 0)  dtostrf(largoInterno[0], 8, 8, value);

      else if (strcmp(buff2, "umbralAlarmaFuerte1") == 0)  dtostrf(umbral_alarma_fuerte[0], 8, 8, value);

      else if (strcmp(buff2, "umbralAlarmaSuave1") == 0)  dtostrf(umbral_alarma_suave[0], 8, 8, value);

      else if (strcmp(buff2, "factorVelocidad1") == 0)  dtostrf(factorVelocidad[0], 8, 8, value);

      else if (strcmp(buff2, "idx_1") == 0)  itoa(idx[0], value, 10);
      
      else if (strcmp(buff2, "th1_1") == 0)  itoa(th1[0], value, 10);
      
      else if (strcmp(buff2, "th1_2") == 0)  itoa(th2[0], value, 10);

      else if (strcmp(buff2, "th1_3") == 0)  itoa(th3[0], value, 10);

      else if (strcmp(buff2, "start_reg_cable_1") == 0)  itoa(registrosCables[0], value, 10);

      else if (strcmp(buff2, "start_reg_cable_fil_1") == 0)  itoa(registrosCablesFiltroMediano[0], value, 10);
      buffFile.write(buff2);
      buffFile.write(": ");
      buffFile.write(value);
      buffFile.write('\r');
      buffFile.write('\n');
    }
    else{
      buffFile.write(original_str);
    }
  }

  // Eliminamos el archivo viejo
  MyFile.close();
  MyFile.open(CONFIG_FILE_NAME, O_WRITE);
  MyFile.remove();

  // Cambiamos el nombre del archivo nuevo
  buffFile.rename(CONFIG_FILE_NAME);
  buffFile.close();
  
  #ifdef SERIAL0
    Serial.println(F("- Variables guardadas en archivo de configuracion!"));
  #endif
  digitalWrite(PIN_SD, HIGH);
  digitalWrite(PIN_ETHERNET, HIGH);

  guardar_datos_eeprom();
}


//////////////////////////////////////////////////////////////////
///                     - FUNCIONES AUXILIARES -            //////
//////////////////////////////////////////////////////////////////
void remove_last_char(char *d, const char *s){
  for(;*s;++s){
      if (*s != '\n')
          *d++ = *s;
    }
     *d = *s;
}

void remove_space(char *d, const char *s){
    for(;*s;++s){
        if(*s != ' ')
            *d++ = *s;
    }
    *d = *s;
}

void split_array(char *var, char *val, const char*s){
  bool is_var = true;
  for(;*s;++s){
    if(*s == ':'){
      is_var = false;
      ++s;
    }
    if (is_var) {
      *var++ = *s;
      
    }
    else {
      *val++ = *s;
    }
  }
  *var++ = '\0';
  *val++ = '\0';
}

int array_point_counter(const char *s){
  int i = 0;
  for(;*s;++s){
    if (*s == '.') i++;
  }
  return i;
}

void split_ip_array(char *s, uint8_t *ip){
  int i = 0;
  int j = 0;
  int k = 0;
  char str[4];
  while(s[i]){
    if (s[i] != '.'){
      str[j] = s[i];
      j++;
    }
    else{
      str[j] = '\0';
      ip[k] = atoi(str);
      k++;
      str[0] = 0;
      str[1] = 0;
      str[2] = 0;
      j = 0;
    }
    i++;
  }
  ip[k] = atoi(str);
}
