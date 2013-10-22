/*
  Tiny code for manage Sigfox Telecom Design UNB modem
  
  This example code is in the public domain.
  Share, it's happiness !

  Note : average consumption 120 ua under 3 VDC with all modules
  With card(s) : ATmega328 bootloader ATmegaBOOT_168_atmega328_pro_8MHz
                 TD1208 UNB modem on eval board (TD1208 EVB)
                 Adafruit ADS1015 board (I2C 12-bits ADC)
*/

/* library */
// Arduino core
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Adafruit_ADS1015.h>

// some const
#define TEST_LED 13
#define MSG_T_REGISTER 0x01
#define MSG_T_STAT     0x02
#define MSG_T_INDEX    0x03

// some types
union stat_payload_t 
{
  struct
  {
    uint8_t msg_type;
    uint8_t id_var;
    int16_t var_max;
    int16_t var_avg;
    int16_t var_min;
    int16_t var_inst;
  };
  uint8_t byte[10];
};

union index_payload_t 
{
  struct
  {
    uint8_t  msg_type;
    uint8_t  id_var;
    uint32_t index_1;
    uint32_t index_2;
  };
  uint8_t byte[10];
};

struct stat_t
{
  int16_t  val;
  int16_t  min;
  int16_t  max;
  int16_t  avg;
  int32_t  avg_sum;
  uint16_t avg_count;
};

// some prototypes
//int16_t read_current(void);
//int16_t read_power(void);
void update_stat(stat_t *_stat_t);
void reset_stat(stat_t *_stat_t);
void sig_send_stat(uint8_t id_var, stat_t *_stat_t);
void sig_send_index(uint8_t id_var, uint32_t index_1, uint32_t index_2);
void cpu_sleep(void);
void cpu_idle(void);
void delay_idle(unsigned long ms);
uint32_t bswap_32 (uint32_t x);
uint16_t bswap_16(uint16_t x);

// some vars
SoftwareSerial modem(2, 3); // RX, TX
Adafruit_ADS1015 ads1015;  	// ads1015 at default I2C address: 0x48
stat_t i;
uint32_t tick_8s   = 0;
uint8_t debug_mode = 0;
uint32_t index_uah_gene = 0;
uint32_t index_uah_cons = 0;

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
  // init vars
  reset_stat(&i);
  // init external ADC
  ads1015.begin();
  // 2x gain   +/- 2.048V (max voltage) 1 bit = 1mV
  //ads1015.setGain(GAIN_TWO);
  // 16x gain  +/- 0.256V (max voltage) 1 bit = 0.125mV
  ads1015.setGain(GAIN_SIXTEEN);  
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
  fprintf_P(&console_out, PSTR("PRESS \"r\" key to reroute to UNB modem\r\n"));
  fprintf_P(&console_out, PSTR("PRESS \"d\" key to set debug mode\r\n"));
  fprintf_P(&console_out, PSTR("wait 4s...\r\n"));
  uint8_t k_loop = 0;
  char key;
  while (k_loop++ < 4)
  {
    delay_idle(1000);
    key = Serial.read();
    if (key != -1) break;
  }
  // check user choice : debug
  if (key == 'd')
  {
    debug_mode = 1;
    fprintf_P(&console_out, PSTR("debug mode on\r\n"));
  }
  // check user choice : reroute
  else if (key == 'r') 
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
  // reset UNB modem
  fprintf_P(&modem_out, PSTR("ATZ\r"));
  delay_idle(4000);
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
}

void loop()
{ 
  // read U on shunt
  int16_t _u_mv = ads1015.readADC_Differential_0_1();
  // two's complement for 12 bits
  if (_u_mv >= 0x0800)
    _u_mv = - ((~_u_mv & 0x07FF) + 1);
  // compute _i in ua, current in 0R025 resistor ()
  // _u is : 1 bit -> 0.125 mv
  int32_t _i_ua =  (int32_t(_u_mv) * 0.125) * 1000 / 0.025;
  // compute _i in ua, current in 10.5 resistor (_u is in mv : 1 bit -> 1 mv) 
  //int32_t _i_ua =  int32_t(_u_mv) * 1000 / 10.5;
  // set uah index
  if (_i_ua > 0)
    index_uah_gene += labs(_i_ua) * 8 / 3600;
  else if (_i_ua < 0)
    index_uah_cons += labs(_i_ua) * 8 / 3600;
  // use i in ma for i stat
  i.val = _i_ua / 1000;
  update_stat(&i);
  // debug mode
  if (debug_mode)
  {
    fprintf_P(&console_out, PSTR("index_gene= %lu uah/index_cons= %lu uah\r\n"),
              index_uah_gene, index_uah_cons);
    fprintf_P(&console_out, PSTR("current= %d ma (max %d, avg %d, min %d)\r\n"),
              i.val, i.max, i.avg, i.min);
    // need for UART delay
    delay_idle(100);
  }
  // send Sigfox frame
  // send index: every 1h (450 * 8s = 3600s) 
  if (tick_8s % 450 == 0)
    sig_send_index(0x01, index_uah_gene, index_uah_cons);
  // send I stat : every 3h (1350 * 8s) with a offset of 8s
  if ((tick_8s + 1) % 1350 == 0)
  {
    sig_send_stat(0x02, &i);
    reset_stat(&i);
  }
  // sleep now...
  cpu_sleep();
  // update tick
  tick_8s++;
}

void update_stat(stat_t *_stat_t)
{
  // first update ?
  if (_stat_t->avg_count == 0)
    _stat_t->min = _stat_t->max = _stat_t->val;
  // is min ?
  else if (_stat_t->val < _stat_t->min)
    _stat_t->min = _stat_t->val;
  // is max ?
  else if (_stat_t->val > _stat_t->max)
    _stat_t->max = _stat_t->val;
  // average compute
  _stat_t->avg_sum += _stat_t->val;
  _stat_t->avg_count++;
  _stat_t->avg = _stat_t->avg_sum/_stat_t->avg_count;
}

void reset_stat(stat_t *_stat_t)
{
  _stat_t->min       = 0;
  _stat_t->max       = 0;
  _stat_t->avg       = 0;
  _stat_t->avg_count = 0;
  _stat_t->avg_sum   = 0;
}

void sig_send_stat(uint8_t id_var, stat_t *_stat_t)
{
  stat_payload_t frame;
  char buffer[30];
  // init var
  memset(&frame, 0, sizeof(frame));
  // set frame field
  frame.msg_type = MSG_T_STAT;
  frame.id_var   = id_var;
  frame.var_max  = bswap_16(_stat_t->max);
  frame.var_avg  = bswap_16(_stat_t->avg);
  frame.var_min  = bswap_16(_stat_t->min);
  frame.var_inst = bswap_16(_stat_t->val);
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
  // time to flush tx buffer
  delay_idle(50);
  // debug message
  if (debug_mode) {
    fprintf_P(&console_out, PSTR("send \"AT$RAW=%s\"\r\n"), buffer);
    delay_idle(50);
  }
}

void sig_send_index(uint8_t id_var, uint32_t index_1, uint32_t index_2)
{
  index_payload_t frame;
  char buffer[30];
  // init var
  memset(&frame, 0, sizeof(frame));
  // set frame field
  frame.msg_type = MSG_T_INDEX;
  frame.id_var   = id_var;
  frame.index_1  = bswap_32(index_1);
  frame.index_2  = bswap_32(index_2);
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
  // time to flush tx buffer
  delay_idle(50);
  // debug message
  if (debug_mode) {
    fprintf_P(&console_out, PSTR("send \"AT$RAW=%s\"\r\n"), buffer);
    delay_idle(50);
  }
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
