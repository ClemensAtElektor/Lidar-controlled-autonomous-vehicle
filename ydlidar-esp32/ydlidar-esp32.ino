/*
 * Purpose: 
 * Board: ESP32 Pico Kit V4
 * 
 * Ydlidar         <-> ESP32 Pico Kit V4
 *  1 (+5V, blue)  <->  5V
 *  2 (TX, green)  <->  IO9 (RX)
 *  3 (RX, yellow) <->  IO10 (TX)
 *  4 (GND, black) <->  GND
 *  5 (M_SCTR, nc) <->  IO18
 *  6 (M_DEV, nc)
 *  7 (M_EN, red)  <->  IO19
 *  8 (NC, nc)
 * 
 * Author: Clemens at Elektor
 * History:
 * - created: 3/3/2021
 * This sketch was created with Arduino IDE 1.8.13.
 */

// Dabble stuff for Bluetooth control.
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// Motor driver.
#include <ESP32MotorControl.h>
ESP32MotorControl mcpwm;
// Motor 1 pins.
const int m0_pin1 = 22;
const int m0_pin2 = 23;
// Motor 2 pins.
const int m1_pin1 = 21;
const int m1_pin2 = 5;

#define FORWARD  (0)
#define BACKWARD  (1)
#define SPIN_LEFT  (2)
#define SPIN_RIGHT  (3)
#define TURN_LEFT  (4)
#define TURN_RIGHT  (5)
bool allow_forward = false;

const int led = 14;
const int ydlidar_en_pin = 19; // Motor enable.
const int ydlidar_sctr_pin = 18; // Motor speed control.

typedef struct __attribute__((__packed__)) 
{
  uint8_t sync_byte1; // 0xa5
  uint8_t sync_byte2; // 0x5a
  uint32_t data_size: 30;
  uint32_t mode: 2;
  uint8_t type;
} 
lidar_system_message_header_t;

typedef struct __attribute__((__packed__)) 
{
  uint8_t sync_byte1; // PH LSB: 0xaa
  uint8_t sync_byte2; // PH MSB: 0x55
  uint8_t type; // CT
  uint8_t sample_count; // LSN
  uint16_t angle_start; // FSA
  uint16_t angle_end; // LSA
  uint16_t checksum; // CS
}
lidar_scan_header_t;

#define LIDAR_RX_BUFFER_SIZE  (128)

typedef struct __attribute__((__packed__))
{
  uint8_t data[LIDAR_RX_BUFFER_SIZE];
  uint8_t index;
}
lidar_rx_buffer_t;

lidar_rx_buffer_t lidar_rx_buffer[2]; // Ping-pong buffer.
uint8_t ping_pong = 0;

int rho_theta[360]; // Table to hold angle-distance (rho-theta) data.

char to_hex[16] = 
{
  '0', '1', '2', '3', '4', '5', '6', '7', 
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', 
};

hw_timer_t *nudge_timer = NULL;

void print_hex(uint8_t ch, uint8_t reset)
{
  static uint8_t i = 0;
  if (reset!=0)
  {
    Serial.println();
    i = 0;
  }
  Serial.print(to_hex[(ch>>4)&0x0f]);
  Serial.print(to_hex[ch&0x0f]);
  Serial.print(' ');
  i++;
  if (i>15)
  {
    Serial.println();
    i = 0;
  }
}

uint16_t checksum(uint8_t ch, uint8_t reset)
{
  static uint16_t crc = 0;
  static uint8_t i = 0;
  if (reset!=0) 
  {
    crc = 0;
    i = 0;
  }
  ((uint8_t*)&crc)[i] ^= ch;
  i ^= 0x01;
  return crc;
}

void dump(uint8_t *p_data, uint16_t data_size)
{
  uint16_t crc = checksum(0,1); // Reset checksum.
  for (uint8_t i=0; i<data_size; i++)
  {
    uint8_t ch = p_data[i];
    crc = checksum(ch,0); // Update CRC.
    print_hex(ch,i==0?1:0);
  }
  Serial.print("\n-> crc=");
  print_hex((crc>>8)&0xff,0);
  print_hex(crc&0xff,0);
  Serial.println();
}

void ydlidar_reset(void)
{
  Serial1.write(0xa5);
  Serial1.write(0x80); // Soft reset.
  uint32_t t_start = millis();
  while (millis()-t_start<500)
  {
    if (Serial1.available())
    {
      char ch = (char)Serial1.read();
      if (ch==0x0d) Serial.println();
      else if (ch!=0x0a) Serial.print(ch);
    }
  }
  Serial.println();
}

void dump_rho_theta(void)
{
  for (int i=0; i<360; i++)
  {
    Serial.print(i);
    Serial.print(",");
    Serial.println(rho_theta[i]);
  }
}

uint8_t ydlidar_command(uint8_t ch)
{
  switch (ch)
  {
    case 'x':
      ch = 0x40; // Restart?
      break;
      
    case 's':
      ch = 0x60; // Start scanning.
      digitalWrite(ydlidar_sctr_pin,1); // Speed is slow.
      digitalWrite(ydlidar_en_pin,1); // Motor on.
      break;

    case 't':
      ch = 0x65; // Stop scanning.
      digitalWrite(ydlidar_en_pin,0); // Motor off.
      Serial1.write(0xa5);
      Serial1.write(ch);
      dump_rho_theta();
      return 1;

    case 'r':
      ydlidar_reset();
      return 1;

    case 'd':
      ch = 0x90; // Device info.
      break;

    case 'h':
      ch = 0x91; // Health info.
      break;

    case 'm':
      digitalWrite(ydlidar_en_pin,0); // Motor off.
      break;

    case 'M':
      digitalWrite(ydlidar_sctr_pin,1); // Speed is slow.
      digitalWrite(ydlidar_en_pin,1); // Motor on.
      break;

    default:
      ch = 0;
  }
  
  if (ch!=0)
  {
    Serial1.write(0xa5);
    Serial1.write(ch);
    return 1;
  }
  return 0; // Unknown command
}

uint8_t ydlidar_receive(uint8_t ch)
{
  static uint8_t ch_prev = 0;
  static uint8_t length_header = 0;
  static uint8_t length_data = 0;
  
  lidar_rx_buffer_t *p_rx_buffer = &lidar_rx_buffer[ping_pong];

  if (ch==0x5a && ch_prev==0xa5)
  {
    // Detected start of a system message.
    // Only three system messages are defined:
    // - health: a5 5a 03 00 00 00 06 00 00 00 (ex., always 10 bytes)
    // - device info: a5 5a 14 00 00 00 04 06 0a 01 01 02 00 02 00 00 06 02 01 00 00 00 00 02 01 05 (ex., always 27 bytes)
    // - scan mode: a5 5a 05 00 00 40 81 (ex., always 7 bytes)
    // Normally, after receiving a scan system message, the device switches to scan mode
    // and continues by sending scan frames (that start with 0xaa55).
    p_rx_buffer->data[0] = ch_prev;
    p_rx_buffer->index = 1;
    length_header = sizeof(lidar_system_message_header_t);
    length_data = 0; // Don't know yet.
  }
  else if (ch==0x55 && ch_prev==0xaa)
  {
    // Detected start of a scan.
    p_rx_buffer->data[0] = ch_prev;
    p_rx_buffer->index = 1;
    length_header = sizeof(lidar_scan_header_t);
    length_data = 0; // Don't know yet.
  }

  // If space allows it, store new byte.
  if (p_rx_buffer->index<LIDAR_RX_BUFFER_SIZE)
  {
    p_rx_buffer->data[p_rx_buffer->index] = ch;
    p_rx_buffer->index += 1;
  }

  // Keep byte for next round.
  ch_prev = ch;

  // Check header reception progress.
  if (length_header>0)
  {
    length_header -= 1;
    if (length_header==0)
    {
      // Header completed.
      if (p_rx_buffer->data[0]==0xa5)
      {
        // We are collecting a System message.
        length_data = ((lidar_system_message_header_t *)p_rx_buffer->data)->data_size;
        // The Length field of a scan command system response message contains 5, yet it has no data...
        if (length_data==5) length_data = 0;
        length_data += 1; // Compensate for data reception progress below.
      }
      else if (p_rx_buffer->data[0]==0xaa)
      {
        // We are collecting Scan data.
        length_data = 2*((lidar_scan_header_t *)p_rx_buffer->data)->sample_count; // A sample is 2 bytes.
        length_data += 1; // Compensate for data reception progress below.
      }
    }
  }

  // Check data reception progress.
  if (length_data>0)
  {
    length_data -= 1;
    if (length_data==0)
    {
      // Frame completed.
      ping_pong ^= 0x01; // Toggle ping pong.
      return 1;
    }
  }

  return 0;
}

uint8_t ydlidar_verify_data(uint8_t *p_data, uint16_t data_size)
{
  uint16_t crc = 0;
  uint8_t i = 0;
  for (int j=0; j<data_size; j++)
  {
    ((uint8_t*)&crc)[i] ^= p_data[j];
    i ^= 0x01;
  }
  return crc==0; // CRC of zero is good.
}

uint8_t ydlidar_extract_angle_distance_data(uint8_t *p_data)
{
  //Serial.println();
  lidar_scan_header_t *p_hdr = (lidar_scan_header_t *)p_data;
  if (p_hdr->type==1)
  {
    // Heading marker, i.e., angle = 0.
    //Serial.print("*****");
    digitalWrite(led,HIGH);
    //delay(50);
    //digitalWrite(led,LOW);
    return 1;
  }
  int lsn = p_hdr->sample_count;
  int fsa = p_hdr->angle_start>>7; // (angle_start>>1)/64 equals >>7
  int lsa = p_hdr->angle_end>>7;
  float angle_dif = lsa - fsa;
  if (angle_dif<0.0) angle_dif += 360.0;
  float da = angle_dif/(lsn-1);
  float ai = fsa;
  if (ai>=360.0) ai -= 360.0;
  else if (ai<0.0) ai += 360.0;
  uint8_t *p = &p_data[sizeof(lidar_scan_header_t)];
  while (lsn>0)
  {
    // Distances are in mm, range is specifed as 10 m, i.e. 10,000 mm, so they fit in an int.
    int distance = *p;
    p++;
    distance += *p * 256;
    p++;
    distance /= 4;
    // if (distance>0) // What to do with distances of 0?
    rho_theta[(int)ai] = (rho_theta[(int)ai]+distance)/2;
    
//    Serial.print(ai);
//    Serial.print(": ");
//    Serial.println(distance);
    
    ai += da;
    if (ai>=360.0) ai -= 360.0;
    else if (ai<0.0) ai += 360.0;
    lsn -= 1;
  }
  return 0;
}

int heading_find_best(int *p_rt_table, uint8_t aperture, int arc_begin, int arc_length)
{
  // On the circle segment from "rho_begin" to "rho_end", find the window with width "aperture"
  // that has the highest sum of values, i.e. the greatest average distance.
  int32_t max = 0;
  int rho = 0;
  for (int i=arc_begin; i<arc_begin+arc_length; i++)
  {
    int32_t sum = 0;
    for (int j=i; j<i+aperture; j++)
    {
      int k = j<360? j : j-360; // Is this faster than j%360 ?
      sum += p_rt_table[k];
    }
    if (sum>max)
    {
      max = sum;
      rho = i;
    }
  }
  rho += aperture/2; // Move to the center of the window.
  if (rho>=360) rho -= 360;
  return rho;
}

void stop_motors(void)
{
  if (mcpwm.isMotorStopped(0)==false) mcpwm.motorStop(0);
  if (mcpwm.isMotorStopped(1)==false) mcpwm.motorStop(1);
}

void IRAM_ATTR nudge_end(void)
{
  mcpwm.motorsStop();
}

void nudge_start(uint32_t us)
{
  //Serial.println("nudging");
  timerAlarmWrite(nudge_timer,us,false); // One shot, delay in microseconds.
  timerWrite(nudge_timer,0); // Make sure the timer starts at 0.
  timerAlarmEnable(nudge_timer); // Enable interrupt.
}

void nudge(uint8_t direction, uint8_t speed)
{
  uint32_t pulse = 50000; // Tweak, pulse duration (in microseconds).
  //uint8_t speed = 15; // Tweak, pulse energy (0-100).
  
  if (timerAlarmEnabled(nudge_timer)==0)
  {
    switch (direction)
    {
      // This all depends, of course, on how the motors are wired.
      case FORWARD: // Forward
        mcpwm.motorReverse(0,speed);
        mcpwm.motorReverse(1,speed);
        nudge_start(pulse);
        break;
        
      case BACKWARD: // Backward
        mcpwm.motorForward(0,speed);
        mcpwm.motorForward(1,speed);
        nudge_start(pulse);
        break;
        
      case SPIN_LEFT: // Spin left using two wheels.
        mcpwm.motorForward(0,speed);
        mcpwm.motorReverse(1,speed);
        nudge_start(pulse);
        break;
        
      case SPIN_RIGHT: // Spin right using two wheels.
        mcpwm.motorForward(1,speed);
        mcpwm.motorReverse(0,speed);
        nudge_start(pulse);
        break;
        
      case TURN_LEFT: // Turn left using one wheel.
        mcpwm.motorReverse(1,speed);
        nudge_start(pulse);
        break;
        
      case TURN_RIGHT: // Turn right using one wheel.
        mcpwm.motorReverse(0,speed);
        nudge_start(pulse);
        break;
  
      default: // Stop
        stop_motors();
    }
  }
}

void dabble_parse(void)
{
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();
  float c = GamePad.getXaxisData();
  float d = GamePad.getYaxisData();

  uint8_t speed = 15; // Tweak, pulse energy (0-100).
  
  if (GamePad.isUpPressed())
  {
    nudge(FORWARD,speed);
  }

  if (GamePad.isDownPressed())
  {
    nudge(BACKWARD,speed);
  }

  if (GamePad.isLeftPressed())
  {
    nudge(SPIN_LEFT,speed);
  }

  if (GamePad.isRightPressed())
  {
    nudge(SPIN_RIGHT,speed);
  }

  if (GamePad.isSquarePressed())
  {
    nudge(TURN_LEFT,speed);
  }

  if (GamePad.isCirclePressed())
  {
    nudge(TURN_RIGHT,speed);
  }

  if (GamePad.isCrossPressed())
  {
    allow_forward = false;
    stop_motors();
    ydlidar_command('t');
  }

  if (GamePad.isTrianglePressed())
  {
    allow_forward = true;
  }

  if (GamePad.isStartPressed())
  {
    ydlidar_command('s');
  }

  if (GamePad.isSelectPressed())
  {
    ydlidar_command('d'); // device info.
  }
}

void setup(void)
{
  memset(lidar_rx_buffer,0,sizeof(lidar_rx_buffer));
  memset(rho_theta,0,sizeof(rho_theta));
  Serial.begin(500000);
  Dabble.begin("LidarEsp32"); // Set Bluetooth name.

  mcpwm.attachMotors(m0_pin1,m0_pin2,m1_pin1,m1_pin2);
  mcpwm.motorsStop();
  nudge_timer = timerBegin(0,80,true); // Timer 0, div by 80 (so run at 1 MHz)
  timerAttachInterrupt(nudge_timer,nudge_end,true); // Attach callback

  Serial1.begin(128000);
  pinMode(led,OUTPUT);
  digitalWrite(led,0);
  pinMode(ydlidar_sctr_pin,OUTPUT);
  digitalWrite(ydlidar_sctr_pin,0);
  pinMode(ydlidar_en_pin,OUTPUT);
  digitalWrite(ydlidar_en_pin,0);
  Serial.println("\nEsp32 Pico Kit listening on Serial 0, sending on Serial 1.");
  ydlidar_reset();
}

void loop(void)
{
  Dabble.processInput();
  dabble_parse();

  // Process data from PC.
  if (Serial.available())
  {
    ydlidar_command(Serial.read());
  }
  
  // Process data from lidar.
  if (Serial1.available())
  {
    if (ydlidar_receive(Serial1.read())!=0)
    {
      lidar_rx_buffer_t *p_rx_buffer = &lidar_rx_buffer[ping_pong==0?1:0];
      if (ydlidar_verify_data(p_rx_buffer->data,p_rx_buffer->index-1)!=0)
      {
        if (ydlidar_extract_angle_distance_data(p_rx_buffer->data)!=0)
        {
          int aperture = 10;
          int heading = heading_find_best(rho_theta,aperture,270,180-aperture); // Only look ahead.
          Serial.println(heading);
          delay(50);
          digitalWrite(led,LOW);
          // TODO: Smooth heading first?
          if (heading<180)
          {
            // On the right.
            if (heading>aperture/2)
            {
              // Turn to the right.
              //nudge(TURN_RIGHT,20);
              nudge(SPIN_RIGHT,15);
            }
            else if (allow_forward)
            {
              // Move forward.
              nudge(FORWARD,15);
            }
          }
          else
          {
            // On the left.
            if (heading<360-aperture/2)
            {
              // Turn to the left.
              //nudge(TURN_LEFT,20);
              nudge(SPIN_LEFT,15);
            }
            else if (allow_forward)
            {
              // Move forward.
              nudge(FORWARD,15);
            }
          }
        }
      }
      //dump(p_rx_buffer->data,p_rx_buffer->index-1);
    }
  }
}
