#include "CoVACIEL.h"
#include "CoVACIEL_Capteur.h"
#define TAILLE_PILE 1500
#define TAILLE_PILE_LIDAR 2048
void CoVACIEL_lidar::setRPlidarCallback()
{
	_lidar.setPostParseCallback(std::bind(&CoVACIEL_lidar::_dataHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void CoVACIEL_lidar::init(uint32_t baudrate, uint8_t RXpin, uint8_t TXpin, int8_t motorPin)
{
	_motorPin = motorPin;
	_lidar.init(baudrate, TXpin, RXpin);
	_lidar.resetLidar();
	setRPlidarCallback();
	if (!_lidar.connectionCheck()) {
		LOG_COVACIEL("connectionCheck() failed");
		while (1);
	}

#ifdef DEBUG
	_lidar.printLidarInfo();
	_lidar.printLidarHealth();
	_lidar.printLidarSamplerate();
	_lidar.printLidarConfig();
	Serial.println();
#endif // DEBUG

	ledcAttachChannel(_motorPin, 25000, 8, 0);
}

void CoVACIEL_lidar::start()
{
	_keepSpinning = true;
	if (_motorPin !=-1)
	{
		_motorStart();
		Serial.println("moteur en route");
	}
	
	_lidar.startStandardScan();
	if (taskHandle == nullptr) {
		xTaskCreate(
			taskFunction,        // Fonction de tâche
			"mesureLidar",            // Nom de la tâche
			TAILLE_PILE_LIDAR,                // Taille de la pile
			this,                // Paramètre de la tâche (pointeur vers l'instance)
			1,                   // Priorité de la tâche
			&taskHandle          // Handle de la tâche
		);
	}

}
void CoVACIEL_lidar::stop()
{
	_keepSpinning = false;
	if (_motorPin != -1)
	{
		_motorStop();
	}
	
	_lidar.stopScan();
	if (taskHandle != nullptr) {
		vTaskDelete(taskHandle);
		taskHandle = nullptr;
	}
}
void CoVACIEL_lidar::refresh()
{
	if (_keepSpinning) {
		int8_t datapointsProcessed = _lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)

		if (datapointsProcessed < 0) {
			_keepSpinning = false;
			_lidar.stopScan();
		} 
	}
}

void CoVACIEL_lidar::_motorStart()
{
	ledcWriteChannel(0, 255 * 60 / 100);
}

void CoVACIEL_lidar::_motorStop()
{
	ledcWriteChannel(0, 0);
}

void CoVACIEL_lidar::_dataHandler(uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality)
{
	/*
	angle_q6 est une valeur sur 15 bits pour un angle allant de 0 a 360°
	la valeur de l'angle est alors angle=angle_q6/64.0
	la valeur entiere est donc equivalente a angle_q6/2^6 ou angle_q6>>6
	*/

	static int secteur = 0; // varie de 0 a 22 ou 23
	//	int intAngle = angle_q6 >> 6;	
	//	int angleSecteur = (intAngle >> 4);// intAngle / 16 -> 22.5 valeurs fournies pour un tour
	int angleSecteur = angle_q6 >> 10;
	if (angleSecteur != secteur)
	{
		secteur = angleSecteur;
		distance[secteur] = dist;
	}
}

void CoVACIEL_lidar::taskFunction(void* parameter)
{
	CoVACIEL_lidar* instance = static_cast<CoVACIEL_lidar*>(parameter);
	while (true)
	{
		instance->refresh();
	}
}

void CoVACIEL_radarUS::init(int sda, int scl,int unite, int address)
{
	Wire.begin(sda, scl, 100000);
	_address = address;
	_distance = 0;
	_unite = unite;
	Wire.beginTransmission(_address);
	Wire.write(CMD_GET_VERSION);
	Wire.endTransmission();

	delay(70); // Attendre que le capteur soit prêt

	Wire.requestFrom(_address, 2); // Demander 2 octets
	if (Wire.available() >= 2) {
		uint8_t highByte = Wire.read();
		uint8_t lowByte = Wire.read();
		_version = highByte; // La version est dans le premier octet
	}
	else _version = -1;  
}

int CoVACIEL_radarUS::getVersion()
{
	return _version;
}

int CoVACIEL_radarUS::getDistance()
{
	if (taskHandle != nullptr) {
		return _distance;
	}
	else return -1;	
}

void CoVACIEL_radarUS::start()
{
	if (taskHandle == nullptr) {
		xTaskCreate(
			taskFunction,        // Fonction de tâche
			"mesureUS",            // Nom de la tâche
			TAILLE_PILE,                // Taille de la pile
			this,                // Paramètre de la tâche (pointeur vers l'instance)
			1,                   // Priorité de la tâche
			&taskHandle          // Handle de la tâche
		);
	}

}

void CoVACIEL_radarUS::stop()
{
	if (taskHandle != nullptr) {
		vTaskDelete(taskHandle);
		taskHandle = nullptr;
	}
}

int CoVACIEL_radarUS::mesure()
{
	// step 1: instruct sensor to read echoes
	Wire.beginTransmission(_address); // transmit to device #112 (0x70)
	// the address specified in the datasheet is 224 (0xE0)
	// but i2c adressing uses the high 7 bits so it's 112
	Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
	Wire.write(byte(_unite));      // command sensor to measure in "inches" (0x50)
	// use 0x51 for centimeters
	// use 0x52 for ping microseconds
	Wire.endTransmission();      // stop transmitting

	// step 2: wait for readings to happen
	//delay(70);                   // datasheet suggests at least 65 milliseconds
	vTaskDelay(pdMS_TO_TICKS(70));

	// step 3: instruct sensor to return a particular echo reading
	Wire.beginTransmission(_address); // transmit to device #112
	Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
	Wire.endTransmission();      // stop transmitting

	// step 4: request reading from sensor
	Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

	// step 5: receive reading from sensor
	if (2 <= Wire.available()) { // if two bytes were received
		_distance= Wire.read();  // receive high byte (overwrites previous reading)
		_distance = _distance << 8;    // shift high byte to be high 8 bits
		_distance |= Wire.read(); // receive low byte as lower 8 bits
		//Serial.println(TAILLE_PILE - uxTaskGetStackHighWaterMark(taskHandle));
		return _distance;
	}
	else {
		return -1; // error
	}	
}

void CoVACIEL_radarUS::taskFunction(void* parameter)
{
	CoVACIEL_radarUS* instance = static_cast<CoVACIEL_radarUS*>(parameter);
	while (true)
	{
		instance->mesure();
	}
}
