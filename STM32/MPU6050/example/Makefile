BINARY = firmware

OPENCM3_DIR=libopencm3
LDSCRIPT = stm32f407vet6.ld

$(shell cp ../../i2c/i2c.* .)
$(shell cp ../mpu6050.c .)

OBJS += usart.o
OBJS += i2c.o
OBJS += main.o
OBJS += mpu6050.o
OBJS += hmc5883l.o
OBJS += madgwick_ahrs.o

include libopencm3.target.mk
