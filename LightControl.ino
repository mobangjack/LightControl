#include <IRremote.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const uint8_t SWITCH_CMD_ON = 0x68; // Key : 0
const uint8_t SWITCH_CMD_OFF = 0x30; // Key : 1

const uint32_t SERVO_ANGLE_SWITCH_OFS = 25;
const uint32_t SERVO_ANGLE_SWITCH_RST = 97;
const uint32_t SERVO_ANGLE_SWITCH_ON = SERVO_ANGLE_SWITCH_RST - SERVO_ANGLE_SWITCH_OFS;
const uint32_t SERVO_ANGLE_SWITCH_OFF = SERVO_ANGLE_SWITCH_RST + SERVO_ANGLE_SWITCH_OFS;

#define SWITCH_DELAY() delay(200)

#define LED_PIN LED_BUILTIN
#define IR_RX_PIN 3
#define BT_RX_PIN 10
#define BT_TX_PIN 11
#define SERVO_PIN 9

#define MIN(A,B) (A<B?A:B)
#define MAX(A,B) (A>B?A:B)

const int SERVO_ANGLE_MIN = MIN(SERVO_ANGLE_SWITCH_ON,SERVO_ANGLE_SWITCH_OFF);
const int SERVO_ANGLE_MAX = MAX(SERVO_ANGLE_SWITCH_ON,SERVO_ANGLE_SWITCH_OFF);;

typedef enum {
  SWITCH_STATE_ON,
  SWITCH_STATE_OFF,
}SwitchState_e;

IRrecv ir(IR_RX_PIN, LED_PIN);
SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo servo;

#define SERVO_ATTACH() servo.attach(SERVO_PIN,SERVO_ANGLE_MIN,SERVO_ANGLE_MAX)
#define SERVO_DETACH() servo.detach()
#define SERVO_ATTACHED() servo.attached()
#define SWITCH_ON() servo.write(SERVO_ANGLE_SWITCH_ON)
#define SWITCH_OFF() servo.write(SERVO_ANGLE_SWITCH_OFF)
#define SWITCH_RST() servo.write(SERVO_ANGLE_SWITCH_RST)
#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)
#define LED_TOG() digitalWrite(LED_PIN, !digitalRead(LED_PIN))

decode_results results;
uint8_t irKeyCode = 0;
uint8_t btKeyCode = 0;
uint8_t spKeyCode = 0;

#define MYPRINT(MSG) do { \
  Serial.print(MSG); \
  bt.print(MSG); \
} while(0)

#define MYPRINT_FMT(MSG,FMT) do { \
  Serial.print(MSG,FMT); \
  bt.print(MSG,FMT); \
} while(0)

#define MYPRINTLN(MSG) do { \
  Serial.println(MSG); \
  bt.println(MSG); \
} while(0)

#define MYPRINTLN_FMT(MSG,FMT) do { \
  Serial.println(MSG,FMT); \
  bt.println(MSG,FMT); \
} while(0)

void myprint(const char* msg)
{
  MYPRINT(msg);
}

void myprint(uint32_t val, int fmt = DEC)
{
  MYPRINT_FMT(val, fmt);
}

void myprintln(const char* msg)
{
   MYPRINTLN(msg);
}

void myprintln(uint32_t val, int fmt = DEC)
{
  MYPRINTLN_FMT(val, fmt);
}

void startup() {
  SERVO_ATTACH();
  SWITCH_RST();
  SWITCH_DELAY();
  SERVO_DETACH();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  bt.begin(9600);
  ir.enableIRIn(); // Start the IR receiver
}

void welcome() {
  myprintln("Welcome, Jack.");
  myprint("/*********************");
  myprint("SYSTEM INFOMATION");
  myprintln("*********************/");
  myprint("IR_RX_PIN: ");
  myprintln(IR_RX_PIN);
  myprint("BT_RX_PIN: ");
  myprintln(BT_RX_PIN);
  myprint("BT_TX_PIN: ");
  myprintln(BT_TX_PIN);
  myprint("SERVO_PIN: ");
  myprintln(SERVO_PIN);
  myprint("SWITCH_CMD_ON: ");
  myprintln(SWITCH_CMD_ON, HEX);
  myprint("SWITCH_CMD_OFF: ");
  myprintln(SWITCH_CMD_OFF, HEX);
  myprint("SERVO_ANGLE_SWITCH_ON: ");
  myprintln(SERVO_ANGLE_SWITCH_ON);
  myprint("SERVO_ANGLE_SWITCH_RST: ");
  myprintln(SERVO_ANGLE_SWITCH_RST);
  myprint("SERVO_ANGLE_SWITCH_OFF: ");
  myprintln(SERVO_ANGLE_SWITCH_OFF);
}

void setup()
{
  startup();
  welcome();
}

void showSwitchState(SwitchState_e switchState)
{
  if (switchState == SWITCH_STATE_ON) {
    myprintln("Switch state: on");
  } else if (switchState == SWITCH_STATE_OFF) {
    myprintln("Switch state: off");
  } 
}

void setSwitchState(SwitchState_e switchState)
{
  if (switchState == SWITCH_STATE_ON) {
    SERVO_ATTACH();
    SWITCH_ON();
    SWITCH_DELAY();
    SWITCH_RST();
    SWITCH_DELAY();
    switchState = SWITCH_STATE_ON;
    SERVO_DETACH();
  } else if (switchState == SWITCH_STATE_OFF) {
    SERVO_ATTACH();
    SWITCH_OFF();
    SWITCH_DELAY();
    SWITCH_RST();
    SWITCH_DELAY();
    switchState = SWITCH_STATE_OFF;
    SERVO_DETACH();
  }
  showSwitchState(switchState);
}

void switchStateCmd(uint8_t cmd)
{
  if (cmd == SWITCH_CMD_ON) {
    setSwitchState(SWITCH_STATE_ON);
  } else if (cmd == SWITCH_CMD_OFF) {
    setSwitchState(SWITCH_STATE_OFF);
  }
}

void blinkLed() {
  static uint32_t count = 0;
  if (++count == 10000) {
    count = 0;
    LED_TOG();
  }
}

void loop() {
  if (ir.decode(&results)) {
    if (results.bits == 32) {
      irKeyCode = (results.value >> 8) & 0xFF;
      if (irKeyCode + ((uint8_t)(results.value & 0xFF)) == 0xFF) {
        myprint("IR key code received: ");
        myprintln(irKeyCode, HEX);
        switchStateCmd(irKeyCode);
      }
    }
    ir.resume(); // Receive the next value
  }
  if (bt.available()) {
    btKeyCode = bt.read();
    myprint("BT key code received: ");
    myprintln(btKeyCode, HEX);
    switchStateCmd(btKeyCode);
    while (bt.available()) bt.read();
  }
  if (Serial.available()) {
    spKeyCode = Serial.read();
    myprint("SP key code received: ");
    myprintln(spKeyCode, HEX);
    switchStateCmd(spKeyCode);
    while (Serial.available()) Serial.read();
  }
  blinkLed();
}
