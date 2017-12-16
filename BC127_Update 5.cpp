#include <SparkFunbc127.h>
#include <SoftwareSerial.h>
#define SINK 0
#define SOURCE 1
// digital inputs w/ interupts (all active low)
const char BTN_VUP = 0;
const char INT_VUP = 2;
const char BTN_VDN = 1;
const char INT_VDN = 3;
const char BTN_MUTE = 2;
const char INT_MUTE = 1;
const char DETECT_OUT = 3;
const char INT_OUT = 0;
const char DETECT_IN = 7;
const char INT_IN = 4;
// outputs
const char MUTE = 8; // digital, active low
const char LED_SINK = 9; // analog, active high
const char LED_SOURCE = 10; // analog, active high
// variables to hold different states
volatile char VUPstate = 0;
volatile char VDNstate = 0;
volatile char MUTEstate = 0;
volatile char DETECT_INstate = 0;
volatile char DETECT_OUTstate = 0;
volatile char CONNstate = 0; // variable tracking connection status
volatile char TIMERstate = 0;
volatile char ROLE = SINK; // variable tracking sink (0) or source (1)
// other variables
int button_hold = 0;
char determined_mode = SINK;
String buffer;
// Create a software serial port.
SoftwareSerial swPort(15, 18); // RX, TX
// create a BC127 and attach the software serial port to it.
BC127 BTModu(&swPort);
void setup()
{
 Serial.begin(9600);

 // Serial port configuration. The software port should be at 9600 baud, as that
 // is the default speed for the BC127.
 swPort.begin(9600);
 // input setup
 pinMode(BTN_VUP, INPUT);
 attachInterrupt(INT_VUP, ISR_VUP, FALLING);
 pinMode(BTN_VDN, INPUT);
 attachInterrupt(INT_VDN, ISR_VDN, FALLING);
 pinMode(BTN_MUTE, INPUT);
 attachInterrupt(INT_MUTE, ISR_MUTE, FALLING);
 pinMode(DETECT_IN, INPUT);
 attachInterrupt(INT_IN, ISR_IN, CHANGE);
 pinMode(DETECT_OUT, INPUT);
 attachInterrupt(INT_OUT, ISR_OUT, CHANGE);
 // output setup
 pinMode(MUTE, OUTPUT);
 pinMode(LED_SINK, OUTPUT);
 pinMode(LED_SOURCE, OUTPUT);
 // unmute
 digitalWrite(MUTE, 1);
 delay(10);
 Serial.println(BTModu.battConfig());
 Serial.println(BTModu.autoConnect());
 Serial.println(BTModu.battStatus());

 // determine module mode and set corresponding LEDs
 determined_mode = BTModu.determineMode();
 if (determined_mode == SINK) {
 Serial.println("Determined mode: sink");
 ROLE = SINK;
 digitalWrite(LED_SOURCE, 0);
 digitalWrite(LED_SINK, 1);
 } else if (determined_mode == SOURCE) {
 Serial.println("Determined mode: source");
 ROLE = SOURCE;
 digitalWrite(LED_SOURCE , 1);
 digitalWrite(LED_SINK, 0);
 } else {
 sinkSetup();
 }
 //set timer1 interrupt at 1Hz
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1B = 0;// same for TCCR1B
 TCNT1 = 0;//initialize counter value to 0
 // set compare match register for 1hz increments
 OCR1A = 6000;// = (8*10^6) / (freq*1024) - 1 (must be <65536)
 // turn on CTC mode
 TCCR1B |= (1 << WGM12);
 // Set CS10 and CS12 bits for 1024 prescaler
 TCCR1B |= (1 << CS12) | (1 << CS10);
 // enable timer compare interrupt
 TIMSK1 |= (1 << OCIE1A);

 sei(); // enable interrupts
}
void loop()
{
 // catch all UART traffic when no hardware interrupts are issued
 if (swPort.available()>0){
 char temp = "";
 
  while (swPort.available()>0){
 temp = swPort.read();
 buffer.concat(temp);
 delay(1);
 }
 }
 // print buffer if not empty
 if (buffer != ""){
 Serial.print("buffer:");
 Serial.println(buffer);
 //Serial.println(buffer.length());
 }
 // if UART message is ABS_VOL 11 0, switch mode to sink
 if (buffer == "ABS_VOL 11 0\r"){
 Serial.println("Command from other BC127 to switch to sink");
 switchToSink();
 }
 // if UART message is ABS_VOL 11 1, switch mode to source
 if (buffer == "ABS_VOL 11 8\r"){
 Serial.println("Command from other BC127 to switch to source");
 switchToSource();
 }
 // if UART message starts with AVRCP_STOP... set TIMERSTATE == 1 right away (instead of waiting for
interrupt)
 if (buffer.startsWith("AVRCP_STOP")){
 Serial.println("Link loss");
 TIMERstate = 1;
 // reset timer
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1B = 0;// same for TCCR1B
 TCNT1 = 0;//initialize counter value to 0
 OCR1A = 4000;
 // turn on CTC mode
 TCCR1B |= (1 << WGM12);
 // Set CS10 and CS12 bits for 1024 prescaler
 TCCR1B |= (1 << CS12) | (1 << CS10);
 // enable timer compare interrupt
 TIMSK1 |= (1 << OCIE1A);
 }

 buffer = ""; // clear buffer
 // every timer
 if (TIMERstate == 1)
 {
 Serial.println("timer looped");
 //Serial.println(BTModu.battStatus());
 // if connected, interrupt every 5 seconds
 if (BTModu.connectionState() == 1){
 Serial.println("Connected!");
 CONNstate = 1;
 OCR1A = 40000;

 // ensure LED is on
  if (ROLE == SINK) digitalWrite(LED_SINK, 1);
 if (ROLE == SOURCE) {
 digitalWrite(LED_SOURCE, 1);
 Serial.println(BTModu.musicCommands(BC127::PLAY));
 }
 // if not connected, interrupt every ~0.5s
 } else {
 Serial.println("Not connected!");
 CONNstate = 0;
 OCR1A = 4000;

 // blink LED and attempt to open connection if configured as source
 if (ROLE == SINK){
 Serial.println("timer sink");
 if (digitalRead(LED_SINK) == HIGH) digitalWrite(LED_SINK, 0);
 else digitalWrite(LED_SINK, 1);
 }
 if (ROLE == SOURCE){
 Serial.println("timer source");

 Serial.println(BTModu.connect("20FABB020188", BC127::A2DP));
 //Serial.println(BTModu.connect("20FABB020189", BC127::A2DP));
 Serial.println(BTModu.musicCommands(BC127::PLAY));
 if (digitalRead(LED_SOURCE) == HIGH) {
 digitalWrite(LED_SOURCE, 0);
 } else {
 digitalWrite(LED_SOURCE, 1);
 }

 }
 }
 TIMERstate = 0;
 }

 // if VUP switch is pressed, increase volume
 if (VUPstate == 1) {
 delay(200);
 Serial.println("VUP pressed");
 Serial.println(BTModu.getVolume());
 digitalWrite(MUTE, 1);
 Serial.println(BTModu.musicCommands(BC127::UP));
 VUPstate = 0;
 }

 // if VDN switch is pressed, decrease volume
 if (VDNstate == 1) {
 delay(200);
 Serial.println("VDN pressed");
 Serial.println(BTModu.getVolume());
 if (BTModu.getVolume() == 2) digitalWrite(MUTE, 0);
 else {
 Serial.println(BTModu.musicCommands(BC127::DOWN));
 digitalWrite(MUTE, 1);
 }
 VDNstate = 0;
 }
  // if MUTE switch is pressed, mute audio driver
 // if MUTE is held down for 1s, switch audio mode
 if (MUTEstate == 1) {
 //delay(200);
 while (digitalRead(BTN_MUTE) == 0 && button_hold < 1000){
 delay(1);
 button_hold++;
 }
 if (button_hold == 1000) {
 Serial.println("Mode change");
 determined_mode = BTModu.determineMode();
 if (determined_mode == SINK) {
 digitalWrite(LED_SINK, 0);
 digitalWrite(LED_SOURCE, 1);
 Serial.println(BTModu.switchToSinkMessage());
 Serial.println(BTModu.switchToSinkMessage());
 Serial.println(BTModu.switchToSinkMessage());
 sourceSetup();
 }
 else if (determined_mode == SOURCE) {
 digitalWrite(LED_SINK, 1);
 digitalWrite(LED_SOURCE, 0);
 Serial.println(BTModu.switchToSourceMessage());
 Serial.println(BTModu.switchToSourceMessage());
 Serial.println(BTModu.switchToSourceMessage());
 sinkSetup();
 }
 else {
 Serial.println(BTModu.switchToSourceMessage());
 Serial.println(BTModu.switchToSourceMessage());
 Serial.println(BTModu.switchToSourceMessage());
 sinkSetup();
 }
 //delay(500);
 }
 else if (button_hold >= 2) {
 if (digitalRead(MUTE) == 1) {
 Serial.println("MUTE");
 digitalWrite(MUTE, 0);
 } else {
 Serial.println("UNMUTE");
 digitalWrite(MUTE, 1);
 }
 }

 button_hold = 0;
 MUTEstate = 0;
 }
 if (DETECT_OUTstate == 1) {
 delay(500);
 if (digitalRead(DETECT_OUT) == 1) {
 Serial.println("DETECT_OUT removed");
 digitalWrite(MUTE, 0);
 } else {
 Serial.println("DETECT_OUT inserted");
 digitalWrite(MUTE, 1);
 }
 DETECT_OUTstate = 0;
  }
 if (DETECT_INstate == 1) {
 delay(500);
 if (digitalRead(DETECT_IN) == 1) {
 Serial.println("DETECT_IN removed");
 } else {
 Serial.println("DETECT_IN inserted");
 }
 DETECT_INstate = 0;
 }
}
void getBTAddress()
{
 char index = 0;
 //String address;
 Serial.println("Getting address:");
 Serial.println(BTModu.getAddress(index, address));
 Serial.println(address);
}
void system_restore()
{
 Serial.println(BTModu.restore());
}
void switchToSink(){
 digitalWrite(MUTE, 0);
 if (ROLE == SOURCE) sinkSetup();
 digitalWrite(MUTE, 1);
}
void switchToSource(){
 digitalWrite(MUTE, 0);
 if (ROLE == SINK) sourceSetup();
 digitalWrite(MUTE, 1);
}
// setup for sink
void sinkSetup()
{
 Serial.println("Setting up sink...");
 Serial.println(BTModu.setClassicSink());
 Serial.println(BTModu.writeConfig());
 Serial.println(BTModu.reset());
 digitalWrite(LED_SOURCE, 0);
 digitalWrite(LED_SINK, 1);
 //Serial.println(BTModu.musicCommands(BC127::PLAY));
 ROLE = SINK;
 // reset timer
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1B = 0;// same for TCCR1B
 TCNT1 = 0;//initialize counter value to 0
 OCR1A = 4000;
 // turn on CTC mode
 TCCR1B |= (1 << WGM12);
 // Set CS10 and CS12 bits for 1024 prescaler
 TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
 TIMSK1 |= (1 << OCIE1A);
}
void sourceSetup()
{
 Serial.println("Setting up source...");
 Serial.println(BTModu.setClassicSource());
 Serial.println(BTModu.writeConfig());
 Serial.println(BTModu.reset());
 digitalWrite(LED_SOURCE, 1);
 digitalWrite(LED_SINK, 0);
 //Serial.println(BTModu.musicCommands(BC127::PLAY));
 ROLE = SOURCE;
 // reset timer
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1B = 0;// same for TCCR1B
 TCNT1 = 0;//initialize counter value to 0
 OCR1A = 4000;
 // turn on CTC mode
 TCCR1B |= (1 << WGM12);
 // Set CS10 and CS12 bits for 1024 prescaler
 TCCR1B |= (1 << CS12) | (1 << CS10);
 // enable timer compare interrupt
 TIMSK1 |= (1 << OCIE1A);
}
//ISRs
void ISR_VUP() {
 VUPstate = 1;
}
void ISR_VDN() {
 VDNstate = 1;
}
void ISR_MUTE() {
 MUTEstate = 1;
}
void ISR_IN() {
 DETECT_INstate = 1;
}
void ISR_OUT() {
 DETECT_OUTstate = 1;
}
ISR(TIMER1_COMPA_vect) {
 TIMERstate = 1;
}