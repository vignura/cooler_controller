/***********************************************************************************************/
/*
 *  \file       : cooler-controller-1V00.ino
 *  \date       : 06-FEB-2022 
 *  \author     : Vignesh S 
 *  \email      : vignura@gmail.com
 *  \copyright  : All Rights are Reserved 
/*
/***********************************************************************************************/
// headers
#include <Relay.h>
// include library for temperature sensor 
#include <OneWire.h>
#include <DallasTemperature.h>
// include library for RTC 
#include <Wire.h>
#include "RTClib.h"

enum week {Sun = 0, Mon, Tue, Wed, Thu, Fri, Sat};
/*************************************** Pin Mappings ******************************************/
// Relay 
#define RELAY_01        7
#define RELAY_02        8
#define MAX_RELAY_COUNT 2

/* Temperature Sensor DS18B20 */
#define TEMP_SENSE_PIN  2
#define ONE_WIRE_BUS    TEMP_SENSE_PIN
/* 
* Magnetic door Sensor MC-38 
* Note: In PULL UP configuration 
* door open   - HIGH
* door closed - LOW
*/
#define DOOR_SENSE_PIN  3
/* RTC I2C pins */
#define RTC_SDA_PIN     "A4"
#define RTC_SCL_PIN     "A5"
/***********************************************************************************************/

/* comment the below macro to disable debug prints */
#define PRINT_DEBUG
#define MAX_DEBUG_MSG_SIZE                  128
#define DEBUG_BUAD_RATE                     115200

/* cooler states */
#define COOLING_STATE_UNKNOWN   0
#define COOLING_STATE_WAITING   1
#define COOLING_STATE_COOL      2

/* cooling threshold temperatures */
#define LOW_TRSH_TEMP_C        6
#define HIGH_TRSH_TEMP_C       10
/****************************************** globals ********************************************/
/* Relay objects pointers */
#define RLY_COMPRESSOR   0
#define RLY_MOTOR        1
Relay *Rly[MAX_RELAY_COUNT] = {0};
unsigned char g_ucRlyPin[MAX_RELAY_COUNT] = {RELAY_01, RELAY_02};

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

/* cooler state */
uint8_t g_cooler_state = UNKNOWN_STATE;

/* Setup a oneWire instance to communicate with any OneWire devices */
OneWire oneWire(ONE_WIRE_BUS);
/* Pass our oneWire reference to Dallas Temperature */
DallasTemperature temp_sensor(&oneWire);
/* RTC */
RTC_DS1307 rtc;


/***********************************************************************************************/
/*! 
* \fn         :: setup()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function performs the following initializations
*                1) Sets the Relay pins as output and sets its state to low
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void setup() {
  
  Relay_init();

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif

  // init temperature sensor
  temp_sensor.begin();

  // initialize RTC
  if (!rtc.begin())
  {
    debug_println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
   debug_println("RTC is NOT running!");
   while (1);
 }
}

/***********************************************************************************************/
/*! 
* \fn         :: Relay_init()
* \author     :: Vignesh S
* \date       :: 21-DEC-2020
* \brief      :: This function initializes MAX_RELAY_COUNT relay instances with states stored
*                EEPROM if the EEPROM Checksum is valid, else initializes the relay instances
*                to default state (OFF)
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void Relay_init()
{
  uint8_t arrState[MAX_RELAY_COUNT] = {0};
  
  debug_println("Setting all relay states to OFF");
  
  // Relay initialization
  for(int i = 0; i < MAX_RELAY_COUNT; i++)
  {
    Rly[i] = new Relay(g_ucRlyPin[i], false);
    Rly[i]->setState(RELAY_OFF);
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: loop()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This functions is called repeatedly from a infinite for loop in main().
*                It does the following functions
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void loop() {

  bool turn_on_cooler = false;
  bool is_door_open = false;
  float cooler_temp_C = 0;
  
  /* read the temperature */
  cooler_temp_C = read_cooler_temp();
  
  /* read door state */
  is_door_open = read_door_state();
  
  /* run our timer rules based RTC */
  turn_on_cooler = run_timer_rules();

  /* if cooler is on, we need to consider other sensor inputs */
  if(turn_on_cooler == true)
  {
    if(should_turn_on_compressor(cooler_temp_C, &g_cooler_state))
    {
      Rly[RLY_COMPRESSOR]->setState(RELAY_ON);
    }
    else
    {
      Rly[RLY_COMPRESSOR]->setState(RELAY_OFF);
    }

    if(is_door_open == true)
    {
      Rly[RLY_MOTOR]->setState(RELAY_OFF);
    }
    else
    {
      Rly[RLY_MOTOR]->setState(RELAY_ON);
    }
  }
  else
  {
    Rly[RLY_COMPRESSOR]->setState(RELAY_OFF);
    Rly[RLY_MOTOR]->setState(RELAY_OFF);
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: run_timer_rules()
* \author     :: Vignesh S
* \date       :: 06-FEB-2022
* \brief      :: This functions is runs our timer rules based on RTC and returns the cooler
*                state 
* \param[in]  :: None
* \param[out] :: None
* \return     :: state
*/
/***********************************************************************************************/
bool run_timer_rules()
{
  bool state = false;
  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

  /* read time for RTC */
  DateTime now = rtc.now();
  uint8_t day = now.day();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  uint8_t second = now.second();

  debug_println("Time: %02d:%02d:%02d Day: %s", hour, minute, second, daysOfTheWeek[day]);
  
  /* day of week based rules */
  if (day != Sun) /* off days */
  {
    state = true;
  }

  /* time of the day based rules */
  if(state == true)
  {
    if(!(hour > 8 && hour < 9))
    {
      state = false;
    }
  }

  debug_println("timer state: %s" state ? "ON" : "OFF");
  return state;
}

/***********************************************************************************************/
/*! 
* \fn         :: read_cooler_temp()
* \author     :: Vignesh S
* \date       :: 06-FEB-2022
* \brief      :: This functions reads the cooler temperature form DS18B20 sensor
*                state 
* \param[in]  :: None
* \param[out] :: None
* \return     :: temp_C
*/
/***********************************************************************************************/
float read_cooler_temp()
{
  float temp_C = 0;

  sensors.requestTemperatures();
  temp_C = sensors.getTempCByIndex(0);
  debug_println("current temperature: %1d.%1d deg C", (int)temp_C, ((temp_C * 10) % 10));

  return temp_C;
}

/***********************************************************************************************/
/*! 
* \fn         :: should_turn_on_compressor()
* \author     :: Vignesh S
* \date       :: 06-FEB-2022
* \brief      :: This functions decides wheather we need to turn on the compressor or not based
*                on the current temperature and cooling state we are in.
*                state 
* \param[in]  :: temp_C
* \param[in]  :: cooling_state
* \param[out] :: cooling_state
* \return     :: temp_C
*/
/***********************************************************************************************/
bool should_turn_on_compressor(float temp_C, uint8_t* cooling_state)
{
  bool state = false;

  if (*cooling_state = COOLING_STATE_COOL)
  {
    if (temp_C > LOW_TRSH_TEMP_C)
    {
      state = true;
    }
    else
    {
      *cooling_state = COOLING_STATE_WAITING;
      state = false;
      debug_println("switching to waiting state");
    }
    debug_println("In cooling state");
  }
  else if (*cooling_state = COOLING_STATE_WAITING)
  {
    if (temp_C < HIGH_TRSH_TEMP_C)
    {
      state = false;
    }
    else
    {
      *cooling_state = COOLING_STATE_COOL;
      state = true;
      debug_println("switching to cooling state");
    }
    debug_println("In waiting state");
  }
  else
  {
    /* our default cooling state */
    *cooling_state = COOLING_STATE_COOL;
  }

  debug_println("compressor state: %s", state ? "ON" : "OFF");
  return state
}


/***********************************************************************************************/
/*! 
* \fn         :: read_door_state()
* \author     :: Vignesh S
* \date       :: 06-FEB-2022
* \brief      :: This functions reads door state form the mounted magnetic sensor
*                state 
* \param[in]  :: None
* \param[out] :: None
* \return     :: state
*/
/***********************************************************************************************/
bool read_door_state()
{
  bool state = false;

  /* assuming pull up configuration */
  if(digitalRead(TEMP_SENSE_PIN) == HIGH)
  {
    state = false;
  }
  else
  {
    state = true;
  }

  debug_println("door state: %s", state ? "OPEN" : "CLOSE");

  return state;
}
/***********************************************************************************************/
/*! 
* \fn         :: PrintBytes()
* \author     :: Vignesh S
* \date       :: 02-DEC-2018
* \brief      :: This function prints the buffer byte by byte 
* \param[in]  :: ucBuffer 
* \param[in]  :: iBuflen
* \return     :: None
*/
/***********************************************************************************************/
void PrintBytes(unsigned char *ucBuffer, int iBuflen)
{
  int iIndex = 0;

  if(ucBuffer == NULL)
  {
    return;
  }

  for(iIndex = 0; iIndex < iBuflen; iIndex++)
  {
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"BY%0d: %#X[%c]", ucBuffer[iIndex]);
      Serial.println(g_arrcMsg);
    #endif
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: debug_println()
* \author     :: Vignesh S
* \date       :: 06-FEB-2022
* \brief      :: This function prints debug strings
* \param[in]  :: string
* \return     :: NONE
*/
/***********************************************************************************************/
void debug_println(const char *format, ...)
{
  #ifdef PRINT_DEBUG    
    va_list pArg;
    va_start(pArg, format);
    _vsntprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, format, pArg);
    va_end(pArg);    
    Serial.println(g_arrcMsg);
  #endif
}