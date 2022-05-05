/**
 * @file MPU6500SPI.cpp
 * @author Jan Wielgus
 */

#include "MPU6500SPI.h"
#include "MPU6050SPIRegisters.h"
#include <SPI.h>

MPU6500SPI::MPU6500SPI(uint32_t spiClock, uint8_t csPin):
	bus(SPI),
	PinCS(csPin),
	spiSettings(spiClock, MSBFIRST, SPI_MODE3)
{
}

MPU6500SPI::MPU6500SPI(SPIClass& spiBus, uint32_t spiClock, uint8_t csPin):
	bus(spiBus),
	PinCS(csPin),
	spiSettings(spiClock, MSBFIRST, SPI_MODE3)
{
}

bool MPU6500SPI::initialize()
{
	pinMode(PinCS, OUTPUT);
	digitalWrite(PinCS, HIGH);

	writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);	// Reset device
	delayMicroseconds(1000); // TODO: test without delay and remove if useless (with different clock speeds)
	writeReg(MPUREG_PWR_MGMT_1, 0x01);			// Clock source
	delayMicroseconds(1000);
	writeReg(MPUREG_PWR_MGMT_2, 0x00);			// Enable Acc & Gyro
	delayMicroseconds(1000);

	writeReg(MPUREG_INT_PIN_CFG, 0x12);			// TODO: check what it does and if is needed
	delayMicroseconds(1000);

	setAccRange(AccRange::Range_2G);
	setGyroScale(GyroScale::Scale250DPS);

	// TODO: setup difital low-pass filter (on chip)

	return true;
}

void MPU6500SPI::readAll()
{
	uint8_t response[14];
	readRegs(MPUREG_ACCEL_XOUT_H, response, 14);

	// Update raw data
	rawAcceleration.x = response[0] << 8 | response[1];
	rawAcceleration.y = response[2] << 8 | response[3];
	rawAcceleration.z = response[4] << 8 | response[5];
	uint16_t rawTemperature = response[6] << 8 | response[7];
	rawRotation.x = response[8] << 8 | response[9];
	rawRotation.y = response[10] << 8 | response[11];
	rawRotation.z = response[12] << 8 | response[13];

	// Apply offsets
	rawAcceleration.x -= accOffset.x;
	rawAcceleration.y -= accOffset.y;
	rawAcceleration.z -= accOffset.z;
	rawRotation.x -= gyroOffset.x;
	rawRotation.y -= gyroOffset.y;
	rawRotation.z -= gyroOffset.z;

	// Calculate normalized
	normAcceleration.x = rawAcceleration.x * accScaleMult;
	normAcceleration.y = rawAcceleration.y * accScaleMult;
	normAcceleration.z = rawAcceleration.z * accScaleMult;
	normRotation.x = rawRotation.x * gyroScaleMult;
	normRotation.y = rawRotation.y * gyroScaleMult;
	normRotation.z = rawRotation.z * gyroScaleMult;

	temperature_degC = ((float)rawTemperature * 0.002941f) + 36.53f; // * 0.002941f is same as / 340
}

void MPU6500SPI::readAcc()
{
	uint8_t response[6];
	readRegs(MPUREG_ACCEL_XOUT_H, response, 6);

	rawAcceleration.x = response[0] << 8 | response[1];
	rawAcceleration.y = response[2] << 8 | response[3];
	rawAcceleration.z = response[4] << 8 | response[5];

	rawAcceleration.x -= accOffset.x;
	rawAcceleration.y -= accOffset.y;
	rawAcceleration.z -= accOffset.z;

	normAcceleration.x = rawAcceleration.x * accScaleMult;
	normAcceleration.y = rawAcceleration.y * accScaleMult;
	normAcceleration.z = rawAcceleration.z * accScaleMult;
}

void MPU6500SPI::readGyro()
{
	uint8_t response[6];
	readRegs(MPUREG_GYRO_XOUT_H, response, 6);

	rawRotation.x = response[0] << 8 | response[1];
	rawRotation.y = response[2] << 8 | response[3];
	rawRotation.z = response[4] << 8 | response[5];

	rawRotation.x -= gyroOffset.x;
	rawRotation.y -= gyroOffset.y;
	rawRotation.z -= gyroOffset.z;

	normRotation.x = rawRotation.x * gyroScaleMult;
	normRotation.y = rawRotation.y * gyroScaleMult;
	normRotation.z = rawRotation.z * gyroScaleMult;
}

void MPU6500SPI::readTemp()
{
	uint8_t response[2];

	readRegs(MPUREG_TEMP_OUT_H, response, 2);
	uint16_t rawTemperature = response[0] << 8 | response[1];
	temperature_degC = ((float)rawTemperature * 0.002941f) + 36.53f; // * 0.002941f is same as / 340
}

float MPU6500SPI::getTemperature_degC()
{
	return temperature_degC;
}

MPU6500SPI::vector3Int16 MPU6500SPI::getAccOffset()
{
	return accOffset;
}

MPU6500SPI::vector3Int16 MPU6500SPI::getGyroOffset()
{
	return gyroOffset;
}

void MPU6500SPI::setAccOffset(vector3Int16 offset)
{
	accOffset = offset;
}

void MPU6500SPI::setGyroOffset(vector3Int16 offset)
{
	gyroOffset = offset;
}

MPU6500SPI::AccRange MPU6500SPI::getAccRange()
{
	return accRange;
}

MPU6500SPI::GyroScale MPU6500SPI::getGyroScale()
{
	return gyroScale;
}

void MPU6500SPI::setAccRange(AccRange range)
{
	uint8_t registerVal;

	switch (range)
	{
	case AccRange::Range_16G:
		accScaleMult = 0.000488281250f; // 1 / 2048 = 0.000488281250f (for other ranges values are in datasheet)
		registerVal = BITS_FS_16G;
		break;
	case AccRange::Range_8G:
		accScaleMult = 0.000244140625f;
		registerVal = BITS_FS_8G;
		break;
	case AccRange::Range_4G:
		accScaleMult = 0.000122070312f;
		registerVal = BITS_FS_4G;
		break;
	case AccRange::Range_2G:
		accScaleMult = 0.000061035156f;
		registerVal = BITS_FS_2G;
		break;
	}

	writeReg(MPUREG_ACCEL_CONFIG, registerVal);
}

void MPU6500SPI::setGyroScale(GyroScale scale)
{
	uint8_t registerVal;

	switch (scale)
	{
	case GyroScale::Scale2000DPS:
		gyroScaleMult = 0.060975609756f;
		registerVal = BITS_FS_2000DPS;
		break;
	case GyroScale::Scale1000DPS:
		gyroScaleMult = 0.030487804878f;
		registerVal = BITS_FS_1000DPS;
		break;
	case GyroScale::Scale500DPS:
		gyroScaleMult = 0.015267175572f;
		registerVal = BITS_FS_500DPS;
		break;
	case GyroScale::Scale250DPS:
		gyroScaleMult = 0.007633587786f;
		registerVal = BITS_FS_250DPS;
		break;
	}

	writeReg(MPUREG_GYRO_CONFIG, registerVal);
}

uint8_t MPU6500SPI::writeReg(uint8_t addr, uint8_t data)
{
	select();
	bus.transfer(addr);
	uint8_t response = bus.transfer(data);
	deselect();

	return response;
}

uint8_t MPU6500SPI::readReg(uint8_t addr)
{
	select();
	bus.transfer(addr | READ_FLAG);
	uint8_t response = bus.transfer(0x00);
	deselect();

	return response;
}

void MPU6500SPI::readRegs(uint8_t addr, uint8_t *readBuf, uint16_t count)
{
	select();
	bus.transfer(addr | READ_FLAG);
	for (uint16_t i = 0; i < count; i++)
	{
		readBuf[i] = bus.transfer(0x00);
	}
	deselect();
}
