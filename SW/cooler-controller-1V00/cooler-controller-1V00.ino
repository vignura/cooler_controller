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

#define PRINT_DEBUG
#include <utility.h>

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
#define TEMP_SENSE_PIN  5
#define ONE_WIRE_BUS    TEMP_SENSE_PIN
/* 
* Magnetic door Sensor MC-38 
* Note: In PULL UP configuration 
* door open   - HIGH
* door closed - LOW
*/
#define DOOR_SENSE_PIN  4
/* RTC I2C pins */
#define RTC_SDA_PIN     "A4"
#define RTC_SCL_PIN     "A5"
/***********************************************************************************************/
#define DEBUG_BUAD_RATE                     115200

/* cooler states */
#define COOLING_STATE_UNKNOWN   0
#define COOLING_STATE_WAITING   1
#define COOLING_STATE_COOL      2

/* cooling threshold temperatures */
#define LOW_TRSH_TEMP_C        6.0f
#define HIGH_TRSH_TEMP_C       10.0f
/****************************************** globals ********************************************/
/* Relay objects pointers */
#define RLY_COMPRESSOR   0
#define RLY_MOTOR        1
Relay *Rly[MAX_RELAY_COUNT] = {0};
unsigned char g_ucRlyPin[MAX_RELAY_COUNT] = {RELAY_01, RELAY_02};

/* cooler state */
uint8_t g_cooler_state = COOLING_STATE_UNKNOWN;

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
  
  // initialize the door sensor pin
  pinMode(DOOR_SENSE_PIN, INPUT_PULLUP);

  Relay_init();

  // Serial port initialization
  #ifdef PRINT_DEBUG
    Serial.begin(DEBUG_BUAD_RATE);
  #endif


  // init temperature sensor
  temp_sensor.begin();

#if 1
  // initialize RTC
  if (!rtc.begin())
  {
    debugPrintln("Couldn't find RTC");
    while (1);
  }
  else
  {
    debugPrintln("RTC Found");
  }

  if (! rtc.isrunning()) 
  {
    debugPrintln("RTC is NOT running!");
    while (1);
  }
  else
  {
    debugPrintln("RTC is running");
  }

#endif
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
  
  debugPrintln("Setting all relay states to OFF");
  
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
  
 #if 0
    // door_sensor_test();
    temp_sensor_test();
 #else 
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
#endif
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

  debugPrintln("Time: %02d:%02d:%02d Day: %s", hour, minute, second, daysOfTheWeek[day]);
  
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

  debugPrintln("timer state: %s", state ? "ON" : "OFF");
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

  temp_sensor.requestTemperatures();
  temp_C = temp_sensor.getTempCByIndex(0);
  debugPrintln("current temperature: %d.%02d deg C", (int)temp_C, ((int)(temp_C * 100) % 100));
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

  if (*cooling_state == COOLING_STATE_COOL)
  {
    if (temp_C > LOW_TRSH_TEMP_C)
    {
      state = true;
    }
    else
    {
      *cooling_state = COOLING_STATE_WAITING;
      state = false;
      debugPrintln("switching to waiting state");
    }
    debugPrintln("In cooling state");
  }
  else if (*cooling_state == COOLING_STATE_WAITING)
  {
    if (temp_C < HIGH_TRSH_TEMP_C)
    {
      state = false;
    }
    else
    {
      *cooling_state = COOLING_STATE_COOL;
      state = true;
      debugPrintln("switching to cooling state");
    }
    debugPrintln("In waiting state");
  }
  else
  {
    /* our default cooling state */
    *cooling_state = COOLING_STATE_COOL;
  }

  debugPrintln("compressor state: %s", state ? "ON" : "OFF");
  return state;
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
  /* when door is open, the sensor is in open 
     circuit, so with a internal pull up, we
     will read HIGH when door is open. */
  if(digitalRead(DOOR_SENSE_PIN) == HIGH)
  {
    state = true;
  }
  else
  {
    state = false;
  }

  debugPrintln("door state: %s", state ? "OPEN" : "CLOSE");

  return state;
}

/***********************************************************************************************/
/*! 
* \fn         :: door_sensor_test()
* \author     :: Vignesh S
* \date       :: 09-FEB-2022
* \brief      :: This function prints the door sensor state continuously
* \param[in]  :: NONE
* \return     :: NONE
*/
/***********************************************************************************************/
void door_sensor_test()
{
  read_door_state();
  delay(500);
}

/***********************************************************************************************/
/*! 
* \fn         :: temp_sensor_test()
* \author     :: Vignesh S
* \date       :: 09-FEB-2022
* \brief      :: This function prints the temperature read from sensor continuously
* \param[in]  :: NONE
* \return     :: NONE
*/
/***********************************************************************************************/
void temp_sensor_test()
{
  read_cooler_temp();
  delay(500);
}
