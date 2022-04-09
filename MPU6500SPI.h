/**
 * @file MPU6500SPI.h
 * @author Jan Wielgus
 */

#ifndef MPU6500SPI_h
#define MPU6500SPI_h

#include <inttypes.h>
#include <Arduino.h>
#include <SPI.h>


class MPU6500SPI
{
public:
	struct vector3Int16 {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	
	struct vector3Float {
		float x;
		float y;
		float z;
	};

	enum class AccRange : uint8_t {
		Range_16G,
		Range_8G,
		Range_4G,
		Range_2G
	};

	enum class GyroScale : uint8_t {
		Scale2000DPS,
		Scale1000DPS,
		Scale500DPS,
		Scale250DPS
	};

private:
	vector3Int16 rawAcceleration;
	vector3Int16 rawRotation;
	vector3Float normAcceleration;	// normalized acceleration
	vector3Float normRotation;		// normalized rotation (gyro data)
	float temperature_degC;

	vector3Int16 accOffset;
	vector3Int16 gyroOffset;

	AccRange accRange;
	GyroScale gyroScale;
	float accScaleMult;			// value to multiply raw reading based on accRange
	float gyroScaleMult;		// value to multiply raw reading based on gyroScale

	// SPI config
	SPIClass& bus;
	const uint8_t PinCS;
	SPISettings spiSettings;


public:
	MPU6500SPI(uint32_t spiClock, uint8_t csPin);
	MPU6500SPI(SPIClass& bus, uint32_t spiClock, uint8_t csPin);

	bool initialize();

	void readAll();
	void readAcc();
	void readGyro();
	void readTemp();

	vector3Int16 getRawAcceleration();
	vector3Int16 getRawRotation();
	vector3Float getNormalizedAcceleration();
	vector3Float getNormalizedRotation();

	float getTemperature_degC();

	vector3Int16 getAccOffset();
	vector3Int16 getGyroOffset();
	void setAccOffset(vector3Int16 offset);
	void setGyroOffset(vector3Int16 offset);

	AccRange getAccRange();
	GyroScale getGyroScale();
	void setAccRange(AccRange range);
	void setGyroScale(GyroScale scale);

	// TODO: calibrate methods??

private:
	/**
	 * @brief Begins the SPI transaction and sets CS pin to LOW.
	 */
	void select();

	/**
	 * @brief Ends the SPI transaction and sets CS pin to HIGH.
	 */
	void deselect();

	uint8_t writeReg(uint8_t addr, uint8_t data);
	uint8_t readReg(uint8_t addr);
	void readRegs(uint8_t addr, uint8_t* readBuf, uint16_t count);
};



inline MPU6500SPI::vector3Int16 MPU6500SPI::getRawAcceleration()
{
	return rawAcceleration;
}

inline MPU6500SPI::vector3Int16 MPU6500SPI::getRawRotation()
{
	return rawRotation;
}

inline MPU6500SPI::vector3Float MPU6500SPI::getNormalizedAcceleration()
{
	return normAcceleration;
}

inline MPU6500SPI::vector3Float MPU6500SPI::getNormalizedRotation()
{
	return normRotation;
}

inline void MPU6500SPI::select()
{
    bus.beginTransaction(spiSettings);
    digitalWrite(PinCS, LOW);
}

inline void MPU6500SPI::deselect()
{
    digitalWrite(PinCS, HIGH);
    bus.endTransaction();
}


#endif
