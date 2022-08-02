#include "vt100.h"
#include "BufferedSerial.h"
#include "mbed.h"
#include <cstdio>

BufferedSerial pc(USBTX, USBRX);

static void vt100_set_white(void) { printf("%c%s", VT100_ESC, VT100_FG_WHITE); }

static void vt100_set_yellow(void) {
  printf("%c%s", VT100_ESC, VT100_FG_YELLOW);
}

void vt100_clear(void) {
  printf("%c%s", VT100_ESC, VT100_HOME);
  printf("%c%s", VT100_ESC, VT100_CLS);
  //printf("%c%s", VT100_ESC, VT100_BOLD);
  printf("%c%s", VT100_ESC, VT100_BG_BLACK);
  vt100_set_yellow();
}

void vt100_init(int baud) {
  pc.set_baud(baud);
  pc.set_format(8, BufferedSerial::None, 1);
  // thread_sleep_for(10);
  vt100_clear();
}

void vt100_set_line_number(int line) {
  char buf[6] = {0};
  int num;
  printf("%c%s", VT100_ESC, VT100_HOME);
  num = sprintf(buf, "[%dB", line);
  // buf[num] = 0; // null termination
  printf("%c%s", VT100_ESC, buf);
  printf("%c%s", VT100_ESC, VT100_CLR);
}

void vt100_print_text(void) {
  vt100_clear();
  printf("*** ABS SIM (12.07.22) ***");
  vt100_set_line_number(2);
  printf("Steering -> (1, 2):");
  vt100_set_line_number(4);
  printf("Car speed (km/h) -> (q, w):");
  vt100_set_line_number(6);
  printf("Wheel speed (km/h):");
  vt100_set_line_number(8);
  printf("Wheel PRR (Hz):");
  vt100_set_line_number(10);
  printf("Wheel circumference (0.4m - 0.9m) -> (4, 5, 6, 7, 8, 9):");
  vt100_set_line_number(12);
  printf("Brake pressure %% -> ([, ]):");
}

void vt100_print_steering_angle(Car c) {
  vt100_set_line_number(3);
  vt100_set_white();
  printf("%3d%%, Angle : %2.2f", c.get_steering(), c.get_steering_angle());
  vt100_set_yellow();
}

void vt100_print_car_speed(Car c) {
  vt100_set_line_number(5);
  vt100_set_white();
  printf("%3d", c.get_default_speed());
  vt100_set_yellow();
}

void vt100_print_wheel_speed(Car c) {
  vt100_set_line_number(7);
  vt100_set_white();
  printf(" Lf : %3.2f, ", c.whl_left_front.get_speed());
  printf("Lr : %3.2f, ", c.whl_left_rear.get_speed());
  printf("Rf : %3.2f, ", c.whl_right_front.get_speed());
  printf("Rr : %3.2f", c.whl_right_rear.get_speed());
  vt100_set_yellow();
}

void vt100_print_prr(Car c) {
  vt100_set_line_number(9);
  vt100_set_white();
  printf(" Lf : %3.2f, ", c.whl_left_front.get_prr());
  printf("Lr : %3.2f, ", c.whl_left_rear.get_prr());
  printf("Rf : %3.2f, ", c.whl_right_front.get_prr());
  printf("Rr : %3.2f", c.whl_right_rear.get_prr());
  vt100_set_yellow();
}

void vt100_print_circumference(Car c) {
  vt100_set_line_number(11);
  vt100_set_white();
  printf(" d: %2.1fm, ", c.get_default_wheel_diameter());
  printf("Lf : %2.2f, ", c.whl_left_front.get_circumference());
  printf("Lr : %2.2f, ", c.whl_left_rear.get_circumference());
  printf("Rf : %2.2f, ", c.whl_right_front.get_circumference());
  printf("Rr : %2.2f", c.whl_right_rear.get_circumference());
  vt100_set_yellow();
}

void vt100_print_brake_pressure(Brake b) {
  vt100_set_line_number(13);
  vt100_set_white();
  printf("%3d", b.get_pressure());
  vt100_set_yellow();
}

int vt100_getc(void) {
  int buf[2] = {0};
  if (pc.readable()) {
    pc.read(buf, 1);
    // pc.write(buf, 1);
    return buf[0];
  }
  return 0;
}
