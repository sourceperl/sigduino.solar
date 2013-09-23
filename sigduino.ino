/*
  Tiny code for manage Sigfox Telecom Design UNB modem
  
  This example code is in the public domain.
  Share, it's happiness !

  Note : average consumption 35 ma on 12VDC. (TODO update this !)
  With card(s) : OLIMEXINO-328 (! 3,3VDC select !)
                 TD1208 UNB modem on eval board (TD1208 EVB)
                 Adafruit ADS1015 board (I2C 12-bits ADC)
*/

/* library */
// Arduino core
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Timer.h>
#include <Adafruit_ADS1015.h>

// some prototypes
void job1();
void jobModem();
void sig_send_var(uint8_t id_var, uint32_t var);
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
Timer t;
int job_1;
int job_modem;
Adafruit_ADS1015 ads1015;  	// ads1015 at default I2C address: 0x48


// link stdout (printf) to Serial object
// create a FILE structure to reference our UART
static FILE console_out = {0};
// create a FILE structure to reference our UNB modem
static FILE modem_out = {0};

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
  ads1015.begin();
  // open serial communications, link Serial to stdio lib
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  modem.begin(9600);
  modem.setTimeout(5000);
  // fill in the UART file descriptor with pointer to writer
  fdev_setup_stream(&console_out, console_putchar, NULL, _FDEV_SETUP_WRITE);
  fdev_setup_stream(&modem_out,   modem_putchar,   NULL, _FDEV_SETUP_WRITE);
  // standard output device STDOUT is console
  stdout = &console_out;
  // init job (with Timer lib)
  job_1      = t.every(1000, job1);
  job1();
  // init job (with Timer lib)
  job_modem  = t.every(3600000, jobModem);
  jobModem();
}

void loop()
{ 
  t.update();
}

// job "modem"
void job1(void)
{
  fprintf_P(&console_out, PSTR("millis()=%lu\r\n"), millis());
}

// job "modem"
void jobModem(void)
{
  sig_send_var(1, millis());
}

void sig_send_var(uint8_t id_var, uint32_t var)
{
  payload_t frame;
  char buffer[40];
  // init var
  memset(&frame, 0, sizeof(frame));
  // set frame field
  frame.msg_type = 0x01;
  frame.id_var   = id_var;
  frame.var      = bswap_32(var);
  // format hex string (ex : 01 01 fe 5e c8 99 00 00...)
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

    
