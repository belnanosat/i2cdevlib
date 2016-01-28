#include <stdio.h>
#include <math.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "mpu6050.h"
#include "usart.h"
#include "madgwick_ahrs.h"
#include "hmc5883l.h"

volatile uint32_t system_microseconds;

void sys_tick_handler(void) {
	system_microseconds++;
}

static void usleep(uint32_t delay) {
	uint32_t wake = system_microseconds + delay;
	while (wake > system_microseconds);
}

static void msleep(uint32_t delay) {
	uint32_t i;
	for (i = 0; i < delay; ++i) usleep(1000);
}

static uint32_t get_time_ms(void) {
	return system_microseconds / 1000;
}

static uint32_t get_time_since(uint32_t start_time) {
	return get_time_ms() - start_time;
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
	/* clock rate / 1000_000 to get 1uS interrupt rate */
	systick_set_reload(168);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_GPIOD);
}

static void getEulerAngles(float *roll, float *pitch, float *yaw) {
	*roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
	*pitch = asin(2.0f * (q0 * q2 - q3 * q1));
	*yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

int main(void)
{
	int i;
	int16_t ax, ay, az, gx, gy, gz;
	float fax, fay, faz, fgx, fgy, fgz;
	int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
	int32_t	sum_gx = 0, sum_gy = 0, sum_gz = 0;
	int16_t gx_offset, gy_offset, gz_offset;
	int16_t mx, my, mz;
	float roll, pitch, yaw;
	clock_setup();
	systick_setup();
	usart_setup();
	i2c_setup();
	msleep(1000);
	MPU6050_initialize();
	if (MPU6050_testConnection()) {
		printf("MPU6050 works!\n\r");
	}
	HMC5883L_Init();
	if (HMC5883L_TestConnection()) {
		printf("HMC5883L works!\n\r");
	}
	printf("Mode: %d\n\r", HMC5883L_GetMode());
	/* ax_offset = MPU6050_getXAccelOffset(); */
	/* ay_offset = MPU6050_getYAccelOffset(); */
	/* az_offset = MPU6050_getZAccelOffset(); */

	/* Init Gyroscope offsets */
	for(i = 0; i < 32; i ++) {
		sum_gx += MPU6050_getRotationX();
		sum_gy += MPU6050_getRotationY();
		sum_gz += MPU6050_getRotationZ();
		msleep(1);
	}
	gx_offset = (sum_gx >> 5);
	gy_offset = (sum_gy >> 5);
	gz_offset = (sum_gz >> 5);

	uint32_t last_update = get_time_ms();
	uint32_t last_print = get_time_ms();
	while (true) {
		if (get_time_since(last_update) < 20) continue;
		MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		HMC5883L_GetHeading(&mx, &my, &mz);
		ax -= ax_offset;
		ay -= ay_offset;
		az -= az_offset;
		gx -= gx_offset;
		gy -= gy_offset;
		gz -= gz_offset;
		fax = ax / 16384.0f;
		fay = ay / 16384.0f;
		faz = az / 16384.0f;
		float ratio = (32767.0 / 250.0f) * 180.0f / M_PI;
		fgx = gx / ratio;
		fgy = gy / ratio;
		fgz = gz / ratio;
		MadgwickAHRSupdate(fgx, fgy, fgz, fax, fay, faz, 0.0f, 0.0f, 0.0f);
		getEulerAngles(&roll, &pitch, &yaw);
		last_update = get_time_ms();
		if (get_time_since(last_print) < 100) continue;
		printf("Acceleration: (%f, %f, %f)\n\r", fax, fay, faz);
		printf("Gyroscope: (%f, %f, %f)\n\r", fgx, fgy, fgz);
		printf("Quaternion: (%f, %f, %f, %f)\n\r", q0, q1, q2, q3);
		printf("Euler angles: Roll: %5f, Pitch: %5f, Yaw: %5f\n\r", roll, pitch, yaw);
		printf("Magnetometer: (%d, %d, %d) %d\n\r", (int)mx, (int)my, (int)mz,
		       (int)HMC5883L_GetLockStatus());
		last_print = get_time_ms();
	}
	return 0;
}
