/*
  Tiny code for manage Sigfox Telecom Design UNB modem
  
  This example code is in the public domain.
  Share, it's happiness !

  Note : average consumption 120 ua under 3 VDC with all modules
  With card(s) : ATmega328 bootloader ATmegaBOOT_168_atmega328_pro_8MHz
                 TD1208 UNB modem on eval board (TD1208 EVB)
                 //Adafruit ADS1015 board (I2C 12-bits ADC)
                 for the moment with BMP085 pressure sensor (for test purpose)
*/

/* library */
// Arduino core
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <avr/sleep.h>
//#include <Adafruit_ADS1015.h>
#include <Adafruit_BMP085.h>

// some prototypes
void read_current(void);
void sig_send_var(uint8_t id_var, uint32_t var);
void cpu_sleep(void);
void cpu_idle(void);
void delay_idle(unsigned long ms);
uint32_t bswap_32 (uint32_t x);
uint16_t bswap_16(uint16_t x);

// some const
#define TEST_LED 13

// some types
union payload_t 
{
  struct
  {
    uint8_t msg_type;
    uint8_t id_var;
    uint32_t var;
  };
  uint8_t byte[10];
};

// some vars
SoftwareSerial modem(2, 3); // RX, TX
uint32_t tick_8s = 0;
Adafruit_BMP085 bmp;
uint32_t p_alt0_max;
uint32_t p_alt0_min;
//Adafruit_ADS1015 ads1015;  	// ads1015 at default I2C address: 0x48

// link stdout (printf) to Serial object
// create a FILE structure to reference our UART
static FILE console_out = {0};
// create a FILE structure to reference our UNB modem
static FILE modem_out = {0};

// *** ISR ***
ISR(WDT_vect)
{
  // do nothing
}

// create serial outputs functions
// This works because Serial.write, although of
// type virtual, already exists.
static int console_putchar (char c, FILE *stream)
{
  Serial.write(c);
  return 0;
}

static int modem_putchar (char c, FILE *stream)
{
  modem.write(c);
  return 0;
}

void setup()
{
  // IO setup
  pinMode(TEST_LED, OUTPUT);
  digitalWrite(TEST_LED, LOW);
  // init external ADC
  //ads1015.begin();
  // 16x gain  +/- 0.256V (max voltage) 1 bit = 0.125mV
  //ads1015.setGain(GAIN_SIXTEEN);  
  // open serial communications, link Serial to stdio lib
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  modem.begin(9600);
  //modem.setTimeout(5000);
  // fill in the UART file descriptor with pointer to writer
  fdev_setup_stream(&console_out, console_putchar, NULL, _FDEV_SETUP_WRITE);
  fdev_setup_stream(&modem_out,   modem_putchar,   NULL, _FDEV_SETUP_WRITE);
  // standard output device STDOUT is console
  stdout = &console_out;
  // set reroute (console <-> UNB modem) ?
  fprintf_P(&console_out, PSTR("PRESS \"x\" key to reroute to UNB modem (wait 5s)\r\n"));
  // wait 5s
  delay_idle(5000); 
  // check user choice
  if (Serial.read() == 'x') 
  {
    // debug mode : reroute console to UNB modem
    fprintf_P(&console_out, PSTR("reroute to UNB modem (reset board to exit this mode)\r\n"));
    while (1) 
    {
      // read from port 0, send to port 1:
      if (Serial.available()) 
        modem.write(Serial.read());
      // read from port 1, send to port 0:
      if (modem.available())      
        Serial.write(modem.read());
    }
  } else {
    // startup message
    fprintf_P(&console_out, PSTR("system start\r\n"));
  }
  // start BMP085
  if (! bmp.begin()) {
	  fprintf_P(&console_out, PSTR("could not find a valid BMP085 sensor, check wiring\r\n"));
	  while (1) {}
  }
  // reset UNB modem
  fprintf_P(&modem_out, PSTR("ATZ\r"));
  delay_idle(1500);
  // init some vars
  p_alt0_max = 0;
  p_alt0_min = 0xFFFFFFFF;
  // *** Setup the watch dog timer ***
  // Clear the reset flag
  MCUSR &= ~(1<<WDRF);
  /* In order to change WDE or the prescaler, we need to
  * set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  // DEBUG: help to send SIGFOX frame
  fprintf_P(&console_out, PSTR("write 's' to console to send SIGFOX frame\r\n"));
  delay_idle(50);
}

void loop()
{ 
  int16_t temp;
  uint16_t pressure_alt0;
  fprintf_P(&console_out, PSTR("tick=%i\r\n"), tick_8s);
  delay_idle(20);
  // call this job every 1 * 8s (8s) 
  if (tick_8s % 1 == 0)   
  {
    temp = bmp.readTemperature() * 10;
    // 0,9... = const for 19 m 
    // see http://fr.wikipedia.org/wiki/Atmosph%C3%A8re_normalis%C3%A9e#Atmosph.C3.A8re_type_OACI)
    //fprintf_P(&console_out, PSTR("p=%u, t=%i\r\n"), pressure_alt0, temp); // copy for debug
    // write 's' to console to send SIGFOX frame
    if (Serial.read() == 's')
    {
      // send frame
      sig_send_var(2, temp);
      // flush buffer
      while(Serial.read() > 0);
    }
    // wait for serial event
    //delay_idle(20);
  }
  // sleep now...
  cpu_sleep();
  // update tick
  tick_8s++;
}

/*
void read_current(void)
{
  int16_t result; 
  result = ads1015.readADC_Differential_0_1();
  // two's complement for 12 bits
  if (result >= 0x0800)
    result = - ((~result & 0x07FF) + 1);
  // to mV (1 bit -> 0,125 mv | 0,125 mv -> 12,5 ma)
  result *=  12.5;
  fprintf_P(&console_out, PSTR("current= %d ma\r\n"), result);
}
*/
void sig_send_var(uint8_t id_var, uint32_t var)
{
  payload_t frame;
  char buffer[30];
  // init var
  memset(&frame, 0, sizeof(frame));
  // set frame field
  frame.msg_type = 0x01;
  frame.id_var   = id_var;
  frame.var      = bswap_16(var);
  // format hex string (ex : 0101fe5ec8990000...)
  strcpy_P(buffer, PSTR(""));
  uint8_t i = 0;
  char digits[3];
  do
  { 
    sprintf(digits, "%02x", frame.byte[i]);
    strcat(buffer, digits);
  } while(++i < sizeof(frame)); 
  // send command
  fprintf_P(&modem_out, PSTR("AT$RAW=%s\r"), buffer);
  fprintf_P(&console_out, PSTR("AT$RAW=%s\r\n"), buffer); // copy for debug
  delay_idle(100);
}

/*** sleep routine ***/ 
void cpu_sleep(void)
{
  // disable uC ADC (enable by arduino core)
  ADCSRA &= ~(1 << ADEN);
  // sleep mode is set here
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  // disable BOD during sleep (auto-reset after wake-up)
  sleep_bod_disable();
  sei();
  // !!! system sleeps here
  sleep_mode();
  // system continues execution here (after watchdog timed out)
  sleep_disable();
  // enable uC ADC
  ADCSRA |= (1 << ADEN);
}


// set uc on idle mode (for power saving) with some subsystem disable
// don't disable timer0 use for millis()
// timer0 overflow ISR occur every 256 x 64 clock cyle
// -> for 16 MHz clock every 1,024 ms, this wake up the "uc" from idle sleep
void cpu_idle(void)
{
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_IDLE);
  power_adc_disable();
  power_spi_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  sleep_mode(); // go sleep here
  sleep_disable(); 
  power_all_enable();
}

// set cpu in idle mode for ms milliseconds
void delay_idle(unsigned long ms)
{
  unsigned long _now = millis();
  while (millis() - _now < ms)
    cpu_idle();
}


// *** swap byte order function for 32 and 16 bits ***
uint32_t bswap_32(uint32_t x)
{
	unsigned char c[4], temp;

	memcpy (c, &x, 4);
	temp = c[0];
	c[0] = c[3];
	c[3] = temp;
	temp = c[1];
	c[1] = c[2];
	c[2] = temp;
	memcpy (&x, c, 4);

	return (x);
}

uint16_t bswap_16(uint16_t x)
{
  return ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8));
}
