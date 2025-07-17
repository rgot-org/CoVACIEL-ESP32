// CoVACIEL_CAN.h
#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include <ArduinoJson.h>
#include <ESP32-TWAI-CAN.hpp>
//#define LOG_COVACIEL Serial.printf

#define CAN_ID_PROPULSION 0x660
#define CAN_ID_VITESSE 0x662
#define CAN_ID_DIRECTION 0x664
#define CAN_ID_AVANT 0x666
#define CAN_ID_ARRIERE 0x667
#define CAN_ID_AVANT_EXT 0x0668

#define DISTANCE_AVANT 11
#define DISTANCE_AV_GAUCHE_45 9
#define DISTANCE_AV_GAUCHE_90 7
#define DISTANCE_AV_DROITE_45 13
#define DISTANCE_AV_DROITE_90 15
#define DISTANCE_AR 0
#define DISTANCE_AR_GAUCHE 20
#define DISTANCE_AR_DROITE 3
#define NB_SECTEURS 23
enum CommandeDirection
{
	gauche,
	droit,
	devant
};
enum CommandePropulsion
{
	accelerer,
	ralentir,
	stop,
	reculer
};
enum Can_id {
	PROPULSION,
	DIRECTION,
	AVANT,
	ARRIERE,
	AVANT_EXT
};

class CoVACIEL_CAN : public TwaiCAN
{
protected:

	CanFrame CANMessages[5];

public:
	/// <summary>
	/// initialise le bus can 
	/// </summary>
	/// <param name="rx">CAN RX PIN</param>
	/// <param name="tx">CAN TX PIN</param>
	/// <returns>true si OK</returns>
	bool initialize(uint8_t rx, uint8_t tx);

	/// <summary>
/// mise a jour des variables apres lecture du bus can
/// </summary>
	void updateRx();
	/// <summary>
	/// envoie les trames sur le bus CAN
	/// </summary>
	/// <param name="canId">CanID : CAN_ID_PROPULSION, CAN_ID_VITESSE, CAN_ID_DIRECTION, CAN_ID_AVANT,CAN_ID_ARRIERE </param>
	/// <returns>true si OK</returns>
	bool updateTx(int canId = 0);

	/// <summary>
	/// retourne la valeur de la propulsion
	/// </summary>
	/// <returns>0:accelerer, 1:ralentir, 2:stop, 3:reculer </returns>
	int getPropulsion();

	/// <summary>
	/// retourne la valeur de la direction
	/// </summary>
	/// <returns>0: Gauche, 1:Droite, 2:Tout droit</returns>
	int getDirection();

	/// <summary>
	/// retourne la distance avant 
	/// </summary>
	/// <returns>distance en mm</returns>
	int getDistanceAvant();

	/// <summary>
	/// retourne la distance avant droite 
	/// </summary>
	/// <returns>distance en mm</returns>
	int getDistAvDroite45();

	/// <summary>
	/// retourne la distance avant gauche 
	/// </summary>
	/// <returns>distance en mm</returns>
	int getDistAvGauche45();

	/// <summary>
	/// retourne la distance arriere 
	/// </summary>
	/// <returns>distance en mm</returns>
	int getDistanceArriere();

	/// <summary>
	/// retourne la vitesse
	/// </summary>
	/// <returns>valeur entre -100 et 100</returns>
	int getVitesse();

	/// <summary>
	/// retourne l'angle de direction
	/// </summary>
	/// <returns>angle en degré</returns>
	int getAngleDirection();

	/// <summary>
	/// retourne une chaine indiquant la propulsion
	/// </summary>
	/// <returns>"accelerer", "ralentir", "stop", "reculer"</returns>
	String getPropulsionStr();

	/// <summary>
	/// retourne une chaine indiquant la direction
	/// </summary>
	/// <returns>"Gauche","Droite","Tout droit"</returns>
	String getDirectionStr();

	int getDistAvDroite90();
	int getDistAvGauche90();
	int getDistanceArGauche();
	int getDistanceArDroite();
	/// <summary>
	/// retourne la distance sur un secteur
	/// </summary>
	/// <param name="secteur">varie de 0 a 22</param>
	/// <returns>distance en mm</returns>
	int getDistance(byte secteur); // secteur est compris entre 0 et 22


	/*********************************************************************/
	/*                SETTERS                                            */
	/*********************************************************************/

	/// <summary>
	/// positionne la directioon
	/// </summary>
	/// <param name="commande">gauche :0, droite: 1, devant: 2</param>
	/// <returns>true si OK</returns>
	bool setDirection(byte commande);





	/// <summary>
/// positionne la propulsion en fonction de la commande :
/// - accelerer (0)
/// - ralentir (1)
/// - stop (2)
/// - reculer(3)
/// </summary>
/// <param name="commande">accelerer,ralentir,stop,reculer</param>
/// <returns>true si OK</returns>
	bool setPropulsion(byte commande);

	
	/// <summary>
	/// definit  la distance avant
	/// </summary>
	/// <param name="mm">distance en mm</param>
	/// <param name="send2canbus">booleen indiquant si la trame doit etre envoyee immediatement</param>
	/// <returns>true si OK</returns>
	bool setDistAv(uint16_t mm, bool send2canbus = false);

	/// <summary>
	/// definit  la distance avant droite a 45°
	/// </summary>
	/// <param name="mm">distance en mm</param>
	/// <param name="send2canbus">booleen indiquant si la trame doit etre envoyee immediatement</param>
	/// <returns>true si OK</returns>
	bool setDistAvDroite45(uint16_t mm, bool send2canbus = false);


	/// <summary>
	/// definit  la distance avant gauche a 45°
	/// </summary>
	/// <param name="mm">distance en mm</param>
	/// <param name="send2canbus">booleen indiquant si la trame doit etre envoyee immediatement</param>
	/// <returns>true si OK</returns>
	bool setDistAvGauche45(uint16_t mm, bool send2canbus = false);

	/// <summary>
	/// definit  la distance arriere
	/// </summary>
	/// <param name="mm">distance en mm</param>
	/// <param name="send2canbus">booleen indiquant si la trame doit etre envoyee immediatement</param>
	/// <returns>true si OK</returns>
	bool setDistAr(uint16_t mm, bool send2canbus = false);


	bool setDistAvDroite90(uint16_t mm, bool send2canbus = false);
	bool setDistAvGauche90(uint16_t mm, bool send2canbus = false);
	bool setDistArDroite(uint16_t mm, bool send2canbus = false);
	bool setDistArGauche(uint16_t mm, bool send2canbus = false);
	/// <summary>
	/// positionne la direction 
	/// </summary>
	/// <param name="angle">angle</param>
	/// <returns>true si OK</returns>
	bool setAngleDirection(int angle);
	/// <summary>
	/// definit les distances a partir d'un tableau de valeurs
	/// </summary>
	/// <param name="distance">nom du tableau (normalement il doit contenir 23 valeurs)</param>
	/// <param name="length">longueur du tableau</param>
	/// <returns></returns>
	bool setDistance(int* distance, byte length);
	/// <summary>
	/// definit les distances à partir d'un objet json
	/// l'objet est un tableau contenant les diferrentes distances sur 23 secteurs
	/// les index des differents secteurs sont definis avec les constantes DISTANCE_AVANT,...,DISTANCE_AR_DROITE
	/// au debut du fichier CoVACIEL_CAN.h
	///	/// </summary>
	/// <param name="jsonDistance"> {"distances":[sect0,sect1,...,sect21,sect22]}</param>
	/// <returns></returns>
	bool setDistanceJson(String jsonDistance);


	/// <summary>
	/// regle la propulsion avec la valeur de vitesse
	/// </summary>
	/// <param name="vitesse">negatif = marche arriere, positif marche avant</param>
	/// <returns></returns>
	bool setVitesse(int vitesse);

	int distance[NB_SECTEURS] = { 0 };
	void canBus2SerialJson();
	String parseCanFrame2json();
	String parseDistancesJson();



private:
	int _lastMessage = 0;

	int _newMessage4Json = 0;
	byte _propulsion = 0;
	byte _direction = 0;
	int _distAvant = 0;
	int _distAvDroite45 = 0;
	int _distAvDroite90 = 0;
	int _distAvGauche45 = 0;
	int _distAvGauche90 = 0;

	int _distAr = 0;
	int _distArDroite = 0;
	int _distArGauche = 0;
	String _parsedDataJson = "";
	int16_t _vitesse;
	int16_t _angle;

	bool sendToCANBus(int can_id);
	void parseData();
};

