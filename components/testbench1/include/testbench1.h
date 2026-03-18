#ifndef _H_TESTBENCH1_H
#define _H_TESTBENCH1_H

#ifdef __cplusplus
extern "C" {
#endif

void blink(void);
void blink_with_timer();
void read_usb_serial();
void test_usb_serial_max_speed();
void battery_read();
void imu_who_am_i();
void imu_read_accel_gyro();
void test_ad_read();

#ifdef __cplusplus
}
#endif
#endif