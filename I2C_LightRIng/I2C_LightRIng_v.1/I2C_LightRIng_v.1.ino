/*********************************************************************************
 * LED Light Ring Controller
 * Developed By: Joey Asher
 * Date: April 27, 2023
 * Revision: 1.4
 * 
 * 
 * This program controls an LED light ring with 3 sections. The 
 * most significant 2 bits of the input byte correspond to the sections on the LED 
 * ring (1, 2, 3, or 0 for all sections), while the remaining 6 bits control the brightness of the LED
 * in that section. The LED brightness is controlled using pulse width modulation (PWM).
 *
 *
 * The program uses a simple interrupt handler when a message is recieved via I2C. 
 * When a byte is received, the most significant 2 bits are found to determine which 
 * section of the LED light ring to control. It then sets the brightness of that 
 * section using the remaining 6 bits of the byte.
 *
 *
 * The LED light ring is connected to the Attiny's I2C pins, I believe it should work on any I2C speed, 
 * but I could be mistaken.
 * 
 * 
 * Written usuing Spence Konde's MegaTinyCore, and programmed using a jtag2udpi converted arduino nano.
 *  
 *  
 * Clock frequency: 8 MHz
 *  
 *********************************************************************************/


#include <Wire.h>

volatile byte interrupt1 = 0; // changed in ISR, so must be volatile or compiker will optimize it away.
volatile byte interrupt2 = 0;

volatile uint32_t buttonPressStartTime = 0;
volatile bool buttonPressed = false;


char pin_ports[] = {0,A3,A4,A5};  // Defining the different sections of the ring
const int button_pin = A2; // Define the button pin
int current_brightness[] = {0,0,0,0};

void start_up(void);



void setup() {
  Wire.begin(0x10); // Start I2C communication with address 0x10
  // Wire.setClock() // This line enables fast mode for the I2C bus 
  Wire.onReceive(recieve_handler); // Set up the I2C handler 

  start_up(); // Call the start up light sequence
  pinMode(PIN_PA2, INPUT); // Set up the button pin to call an interrupt when it is pressed and released
  PORTA.PIN2CTRL  = 0b00000001; //PULLUPEN = 0, ISC = 1 trigger both edges


}


void loop() {
}



void recieve_handler() {
  int msg = Wire.read(); // Read the data
  
  if (msg & 0b11000000) {  // Check to see if the message is being sent to one light section or all 
    int section = (msg & 0b11000000) >> 6;  // Get the section number from the message
    int brightness = (msg & 0b00111111) * 4;  // Get the desired brightness from the message
    
    // Ramp up the brightness gradually
    if(brightness >= current_brightness[section]){
      for (int i = current_brightness[section]; i <= brightness;  i++) {
        analogWrite(pin_ports[section], i);
        delay(5);
      }
    }
    // Ramp the brightness down gradually
    else{
      for (int i = current_brightness[section]; i >= brightness;i--) {
        analogWrite(pin_ports[section], i);
        delay(2);
      }
    }
    current_brightness[section] = brightness;
  }


  else{  // It is being sent to all sections
    int brightness = (msg & 0b00111111) * 4;  // Get the desired brightness from the message
    
    // Ramp up the brightness gradually
    if(brightness >= current_brightness[0]){
      for (int i = current_brightness[0]; i <= brightness;  i++) {
        analogWrite(pin_ports[1], i);
        analogWrite(pin_ports[2], i);
        analogWrite(pin_ports[3], i);
        delay(5);
      }
    }
    
    // Ramp down the brightness gradually
    else{
      for (int i = current_brightness[0]; i >= brightness;i--) {
        analogWrite(pin_ports[1], i);
        analogWrite(pin_ports[2], i);
        analogWrite(pin_ports[3], i);
        delay(2);
      }
    }
    current_brightness[0] = brightness;
  }
}


void start_up() {
  for (int i = 0; i < 385; i++) {
    if (i<=64) {
      analogWrite(pin_ports[1], i);
    }
    else if (i>=65 && i<=128) {
      analogWrite(pin_ports[1], i);
      analogWrite(pin_ports[2], i - 64);
    }
    else if (i>=129 && i<= 192) {
      analogWrite(pin_ports[1], 128 - (i-128));
      analogWrite(pin_ports[2], i - 64);
      analogWrite(pin_ports[3], i - 128);
    }
    else if (i>=193 && i<= 256) {
      analogWrite(pin_ports[1], 128 - (i-128));
      analogWrite(pin_ports[2], 128 - (i - 192));
      analogWrite(pin_ports[3], i - 128);
    }
    else if (i>=257 && i<= 320) {
      analogWrite(pin_ports[2], 128 - (i - 192));
      analogWrite(pin_ports[3], 128 - (i - 256));
    }
    else if (i>= 321) {
      analogWrite(pin_ports[3], 128 - (i - 256));
    }

    delay(3);
  }
}


ISR(PORTA_PORT_vect) {
  if (!digitalRead(button_pin)) {
    buttonPressStartTime = millis();
    buttonPressed = true;
  } else {
    if (buttonPressed) {
      uint32_t buttonPressDuration = millis() - buttonPressStartTime;
      if (buttonPressDuration >= 1000) _PROTECTED_WRITE(RSTCTRL.SWRR,1);  //If the button is held for a long time, reset the micro controller
      else if (buttonPressDuration >= 10) {
        static uint8_t button_brightness = 0;
        if (button_brightness < 7) {
          analogWrite(pin_ports[1], button_brightness * 32 - 1);
          analogWrite(pin_ports[2], button_brightness * 32 - 1);
          analogWrite(pin_ports[3], button_brightness * 32 - 1);
          button_brightness++;
        } else {
          button_brightness = 0;
        }
      }

      buttonPressed = false;
      buttonPressStartTime = 0;
    }
  }

  byte flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags
}
