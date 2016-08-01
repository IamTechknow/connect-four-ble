#ifndef MAIN__H_
#define MAIN__H_

#include "nrf_gpio.h"

//FSM declarations
enum STATES {INIT, CONNECT, CONNECTED, PLAYER1, PLAYER2, SENDDATA, WAIT_FOR_DATA} state;

//Constants
#define BLUE 1
#define RED 4
#define ROWS 16
#define COLS 16
#define WIDTH 16
#define RBG_WIDTH 32
#define REFRESH_TIME 1

//Pin declarations
#define R1 ARDUINO_A0_PIN //RGB bits, rows 0-7
#define G1 ARDUINO_A1_PIN
#define B1 ARDUINO_A2_PIN
#define R2 ARDUINO_A3_PIN //RGB bits, rows 8-15
#define G2 ARDUINO_A4_PIN
#define B2 ARDUINO_A5_PIN
#define A ARDUINO_0_PIN //row select bits
#define B ARDUINO_1_PIN
#define C ARDUINO_2_PIN
#define LAT ARDUINO_3_PIN //latch
#define CLK ARDUINO_4_PIN
#define OE ARDUINO_5_PIN //output enable

#endif
