#pragma once
#include "CoVACIEL_lidar.h" // version modifiee de thijs_rplidar.h
#include <Wire.h>
//#define DEBUG
#define UNITE_CM 0x51
#define UNITE_INCH 0x50
#define CMD_GET_VERSION 0x00 // Commande pour obtenir la version du firmware
#define RADAR_DEFAULT_ADDR 0x70


class CoVACIEL_lidar
{
public:
	CoVACIEL_lidar(HardwareSerial& lidarSerial ) : _lidar(lidarSerial) {} ;// voir https://chat.mistral.ai/chat/77ad5dcb-32f0-4cad-a888-99e5d39cf5f0 
	// Méthode pour définir le callback de RPlidar
	void setRPlidarCallback();
	void init(uint32_t baudrate, uint8_t RXpin, uint8_t TXpin, int8_t motorPin =-1);
	void start();
	void stop();
	void refresh();
	int distance[25] = { 0 };

private:
	RPlidar _lidar;
	int8_t _motorPin;
	void _motorStart();
	void _motorStop();
	bool _keepSpinning = false;
	void _dataHandler(uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality);
	TaskHandle_t taskHandle = nullptr;
	// Fonction de tâche statique
	static void taskFunction(void* parameter);

};
class CoVACIEL_radarUS
{
public:
	/// <summary>
	/// initialise le radar I2C
	/// </summary>
	/// <param name="sda">broche SDA</param>
	/// <param name="scl">broche SCL</param>
	/// <param name="unite">unite : UNITE_CM ou UNITE_INCH (par defaut UNITE_CM)</param>
	/// <param name="address">adresse I2C (par defaut RADAR_DEFAULT_ADDR : 0x70)</param>
	void init(int sda, int scl,  int unite = UNITE_CM,int address = RADAR_DEFAULT_ADDR);
	/// <summary>
	/// retourne la version du micrologiciel du radar
	/// <para>permet de tester si le capteur repond</para>
	/// </summary>
	/// <returns>valeur du firmware</returns>
	int getVersion();
	/// <summary>
	/// renvoit la distance dans l'unite choisie lors de l'initialisation
	/// </summary>
	/// <returns>distance en cm ou inch</returns>
	int getDistance();
	/// <summary>
	/// demarre la tache de mesure
	/// <para>il s'agit d'une tache de fond qui tourne en permanence</para>
	/// </summary>
	void start();
	/// <summary>
	/// stoppe la tache de mesure
	/// </summary>
	void stop();

private:
	int _address;
	int _distance;
	int _unite;
	int _version;
	int mesure();
	TaskHandle_t taskHandle = nullptr;
	// Fonction de tâche statique
	static void taskFunction(void* parameter);

};



