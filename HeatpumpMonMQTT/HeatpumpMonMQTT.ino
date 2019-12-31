#include <WiFi.h>
#include <PubSubClient.h>

#ifndef WIFI_SSID
#define WIFI_SSID "xxxx"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "xxxx"
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER "1.2.3.4"
#endif

#define IOPIN 4

// WIFI and MQTT client globals
WiFiClient    wificlient;
PubSubClient  psclient(wificlient);

////////////////////////////////////////////////////////////////////////////////////////////////

// State machine 
volatile unsigned long  us_last;            // last time we have a GPIO interrupt (microseconds tick)
volatile byte           state;              // state machine state
volatile byte           bit_count;          // Number of bits assembed in current message
volatile byte           bytes[13];          // Buffer of pre-assembled 104 bits

volatile byte           toggle= LOW;        // indicator LED state, flash for each bit
volatile byte           f_start= 0;         // flag: started receiving a message
volatile byte           f_done= 0;          // flag: complete message received, can upload/print message

////////////////////////////////////////////////////////////////////////////////////////////////

// Flags to indicate errors/warning - 0 is disabled, non-zero is value associated with message
volatile byte           m_unknownstate= 0;  // Invalid state detected
volatile byte           m_unexpectlow= 0;   // Unexpected low in state 1
volatile byte           m_unexpecthigh= 0;  // Unexpected high in state 2
volatile unsigned long  t_highidle= 0;
volatile unsigned long  t_highbit_long= 0;
volatile unsigned long  t_highbit_short= 0;
volatile unsigned long  t_lowbit_long= 0;
volatile unsigned long  t_lowbit_short= 0;
volatile unsigned long  t_follow_long= 0;
volatile unsigned long  t_follow_short= 0;
volatile unsigned long  t_init_long= 0;
volatile unsigned long  t_init_short= 0;

////////////////////////////////////////////////////////////////////////////////////////////////

// Interrupt routine for GPIO state change
void IRAM_ATTR io_int()
{
 int           level;                        // Current level of GPIO pin
 unsigned long us_now= micros();             // Current time in microseconds
 unsigned long delta= us_now-us_last;        // Time between now and previous GPIO interrupt
  
 us_last= us_now;                            // Save current time for next interrupt
 level= digitalRead(IOPIN);                  // Save IO pin level
 toggle = !toggle;                           // toggle LED to indicate activity
  
 //////////////////////////////////////////////////////
 if (state==0)
 {                                            // was at idle (high) state between messages
  if (level==LOW)
   state= 1;                                  // falling edge from idle
 }
 //////////////////////////////////////////////////////
 else if (state==1)
 {                                            // was at the start of the initial falling edge
  state= 0;                                   //  assume we have to fall back to state 0
  if (level==LOW)
   m_unexpectlow= 1;                          // input shouldn't be low in state 1
  else if (delta<7000)
   t_init_short= delta;                       // initial low pulse was too short
  else if (delta>12000)
   t_init_long= delta;                        // initial low pulse was too long
  else
   state= 2;                                  // end of initial low (with correct length), now at start of the initial high
 }
 //////////////////////////////////////////////////////
 else if (state==2)
 {                                            // was at the start of the initial rising edge
  state= 0;                                   //  assume we have to fall back to state 0
  if (level==HIGH)
   m_unexpecthigh= 2;                         // input level shouldn't be high in state 2
  else if (delta<2000)    // was 3500
   t_follow_short= delta;                     // high pulse after initial low pulse was too short
  else if (delta>6500)
   t_follow_long= delta;                      // high pulse after initial low pulse was too long
  else
  {                                           // OK we are now at the start of a valid new message
   state= 3;                                  // at the falling edge of the first bit
   f_done= 0;                                 // clear out flags, clear bit counter
   f_start= 1;
   bit_count= 0;
  }
 }
 //////////////////////////////////////////////////////
 else if (state==3)
 {                                            // was at the falling edge (start) of a bit
  state= 0;                                   //  assume we have to fall back to state 0
  if (level==LOW)
   m_unexpectlow= 3;                          // level shouldn't be low in state 3
  else if (delta<700)
   t_lowbit_short= delta;                     // low pulse of bit was too short
  else if (delta>1300)
   t_lowbit_long= delta;                      // low pulse of bit was too long
  else
  {
   state= 4;                                  // at the start of the high period for a bit
   if (bit_count>=104)                        // see if there are more bits left in the message
   {
    state= 5;                                 // no more bits left, we are the start of idle high
    f_done= 1;                                // set flag to indicate this message is done, ready for pick-up
   }
  }
 }
 //////////////////////////////////////////////////////
 else if (state==4)
 {                                            // was at the rising edge of a bit
  state= 0;                                   //  assume we have to fall back to state 0
  if (level==HIGH)
   m_unexpecthigh= 4;                         // level shouldn't be high in state 4
  else if (delta<700)
   t_highbit_short= delta;                    // high pulse of bit was too short
  else if (delta>4200)
   t_highbit_long= delta;                     // high pulse of bit was too long
  else
  {                                           // bit timing is OK, now check if this is a high or low bit
   if (delta<2000)                            // Short is 1 bit value [keypad messages are inverted]
    bytes[bit_count/8]= 0x80 | (bytes[bit_count/8]>>1);   // Shift new 1 bit into current message byte
   else                                       // Long is 0 bit value [but keypad messages are inverted]
    bytes[bit_count/8]= 0x7F & (bytes[bit_count/8]>>1);   // Shift new 0 bit into current message byte
   bit_count++;
   state= 3;                                  // now at the start of a new bit, at the falling edge
  }
 }
 //////////////////////////////////////////////////////
 else if (state==5)
 {                                            // was at the idle high state between messages
  state= 0;                                   //  assume we have to fall back to state 0
  if (level==LOW)
  {
   state= 1;                                  // now at the falling edge from idle, start of a new message
   t_highidle= delta;
  }
 }
 //////////////////////////////////////////////////////
 else
 {
  // printf ("unknown state: %d\n",state);
  state= 0;
 }

 if (state==0)
  toggle= LOW;                               // turn LED off if we are not a valid part of the state machine
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void pscallback (char* topic, byte* message, unsigned int length)
{
 printf ("Message arrived on topic: %s.\n",topic);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
 Serial.begin(115200);
 //Serial.begin(921600);
 Serial.setDebugOutput(true);

printf ("config %s %s %s\n",WIFI_SSID,WIFI_PASSWORD,MQTT_SERVER);

 printf("\nWiFi connected\n");
 printf("IP address: %s\n",WiFi.localIP().toString().c_str());

 // Configure WIFI
 WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
 while (WiFi.status() != WL_CONNECTED)
 {
  delay(500);
  printf(".");
 }

 printf("\nWiFi connected\n");
 printf("IP address: %s\n",WiFi.localIP().toString().c_str());
 
 psclient.setServer(MQTT_SERVER,1883);
 psclient.setCallback(pscallback);

 // Init for message / GPIO interface
 pinMode(IOPIN,INPUT_PULLUP);
 state= 0;
 us_last= micros();
 attachInterrupt(digitalPinToInterrupt(IOPIN),io_int, CHANGE);

 // Initialize the LED_BUILTIN pin as an output
 pinMode(LED_BUILTIN, OUTPUT);

 printf ("GPIO Init done..\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////

#define CHECK(X,Y) { if(X) { printf(Y ": %lu\r\n",X); X=0; } }
unsigned int  last_ms= 0;

void loop()
{
 char         msgbuffer[128];
 unsigned int now_ms= millis();
 byte         f_done_local;
 
 if (!(WiFi.isConnected()))
 {
  printf ("WIFI not connected - attempt reconnection...");
  psclient.disconnect();
  wificlient.stop();
  WiFi.reconnect();
 }

 if (!psclient.connected())
 {
  printf ("MQTT connection broken (state=%d). Attemping new connection...",psclient.state());
  // Attempt to connect
  if (psclient.connect("HeatpumpMon"))
  {
   printf(" connected\n");
   // Subscribe
   psclient.subscribe("heatpump/tx");
  }
  else
  {
   printf (" failed, rc= %d\n",psclient.state());
   delay(1000);
   return;
  }
 }

 // Update display of GPIO interrupts
 digitalWrite(LED_BUILTIN,toggle);

 // Check for any accumated error messages from interrupt routine
 CHECK(m_unknownstate,"Unknown state")
 CHECK(m_unexpectlow,"Unexpected low input in state")
 CHECK(m_unexpecthigh,"Unexpected high input in state")
 CHECK(t_highbit_long,"Highbit too long")
 CHECK(t_highbit_short,"Highbit too short")
 CHECK(t_lowbit_long,"Lowbit too long")
 CHECK(t_lowbit_short,"Lowbit too short")
 CHECK(t_follow_long,"Follow pulse too long") 
 CHECK(t_follow_short,"Follow pulse too short")
 CHECK(t_init_long,"Init pulse too long")
 CHECK(t_init_short,"Init pulse too short") 

 f_done_local= f_done;
 if ((now_ms-last_ms>5000)||f_done_local)
 {
  last_ms= now_ms;
  if (f_done_local)
  {
   sprintf (msgbuffer,"@%lu[%d]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",millis(),WiFi.RSSI(),bytes[0],bytes[1],bytes[2],bytes[3],bytes[4],bytes[5],bytes[6],bytes[7],bytes[8],bytes[9],bytes[10],bytes[11],bytes[12]);
   psclient.publish("heatpump/rx",msgbuffer);
   printf ("%s\n",msgbuffer);
   f_done= 0;
  }
  else
   printf ("@%5lu[%d]: -- no new heatpump message --\n",millis(),WiFi.RSSI());
 }
}
