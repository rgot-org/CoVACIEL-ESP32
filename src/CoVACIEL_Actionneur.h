#pragma once
//#include "CoVACIEL_CAN.h"
#include <Preferences.h>
//
/*-----------============ Driver Moteur ==========-------------*/
#define RESOLUTION 14 // resolution du PWM 14 bits : quantum = 1.2µs
#define SERVO_BASE_FREQ 50
#define DIRECTION_CHANNEL 0
#define PROPULSION_CHANNEL 1
#ifdef ARDUINO_M5STACK_ATOM
#define LED_PIN 27
#elif defined ARDUINO_WAVESHARE_ESP32_S3_ZERO
#define LED_PIN 21
#elif defined ARDUINO_M5STACK_STAMP_C3
#define LED_PIN 2
#elif defined ARDUINO_M5STACK_STAMP_S3
#define LED_PIN 21
#endif // ARDUINO_M5STACK_ATOM

const int pleineEchelle = pow(2, RESOLUTION) - 1;
const int angleDegreMax = 18;

class CoVACIEL_direction
{
public:
	/// <summary>
	/// initialise le pwm du servo direction
	/// </summary>
	/// <param name="pin">GPIO</param>
	void init(int pin);
	/// <summary>
	/// positione la direction à l'angle voulu
	/// </summary>
	/// <param name="value">angle : -18= butee gauche, 0= tout droit, 18= butee droite</param>
	void setDirectionDegre(int value);
	/// <summary>
	/// enregistre la valeur du pwm correspondant a la butee droite
	/// </summary>
	/// <param name="value">valeur PWM</param>
	void setButeeDroite(int value);
	/// <summary>
	/// enregistre la valeur du pwm correspondant a la butee gauche
	/// </summary>
	/// <param name="value">valeur PWM</param>
	void setButeeGauche(int value);
	/// <summary>
	///******************* REGLAGE DES BUTEES *********************
	///* bg pour aller a la butee gauche, bd pour celle de droite *
	///* g pour diminuer la butee gauche, G pour l'augmenter      *
	///* d pour diminuer la butee droite, D pour l'augmenter      *
	///************************************************************
	/// </summary>
	void reglageButees();
	/// <summary>
	/// lit la valeur du PWM pour butee droite
	/// </summary>
	/// <returns>valeur PWM</returns>
	int getButeeDroite();
	/// <summary>
	/// lit la valeur du PWM butee gauche
	/// </summary>
	/// <returns>valeur PWM</returns>
	int getButeeGauche();
	/// <summary>
	/// efface tous le parametres de butee
	/// </summary>
	void clearParams();
private:
	const float rapportCycliqueCentre = pleineEchelle * 6.9f / 100;
	const float rapportCycliqueMax = pleineEchelle * 7.7f / 100;
	const float rapportCycliqueMin = pleineEchelle * 6.1f / 100;
	float _buteeDroite = rapportCycliqueMax;
	float _buteeGauche = rapportCycliqueMin;
	float _centre = rapportCycliqueCentre;
};



class CoVACIEL_propulsion 
{
public:
	/// <summary>
	/// initialise le PWM du driver de prolpulsion
	/// </summary>
	/// <param name="pin">GPIO</param>
	void init(int pin);
	/// <summary>
	/// regle la valeur de la propulsion
	/// </summary>
	/// <param name="value">valeur de -100 a  +100 (%)</param>
	void setPropulsion(int value);// value de -100 à +100 (%)
	/// <summary>
	/// renvoie la valeur de la propulsion actuelle
	/// </summary>
	/// <returns></returns>
	int getPropulsion();
	/// <summary>
	/// regle le PWM pour une propulsion a 100%
	/// </summary>
	/// <param name="value">valeur PWM</param>
	void setPWMAvMax(int value);
	/// <summary>
	/// regle le PWM pour une propulsion a 0% (arret)
	/// </summary>
	/// <param name="value">valeur PWM</param>
	void setPWMAvMin(int value);
	/// <summary>
	/// regle le PWM pour une propulsion a -100%
	/// </summary>
	/// <param name="value">valeur PWM</param>
	void setPWMArMax(int value);
	int getPWMAvMax();
	int getPWMArMax();
	int getPWMAvMin();
	/// ******************* REGLAGE DES BUTEES *********************
	/// * Necessite Serial.begin(...) dans setup()                 *
    /// * M pour aller a la PWM MAX, m pour la PWM min     *
    /// * d pour diminuer la PWM MAX, D pour l'augmenter       *
    /// * r pour diminuer la PWM ar MAX, R  pour l'augmenter   *
	/// * n pour diminuer la PWM 0, N pour l'augmenter         *	
    /// ************************************************************
	void reglageButees();
	/// <summary>
	/// effacement des parametres de PWM
	/// </summary>
	void clearParams();

private:
	/*----------======== Constantes =======----------*/
	
	const char arret = 0;
	const float rapportCycliqueFrein =  pleineEchelle * 5.0 / 100;
	const float rapportCycliqueArret = pleineEchelle * 7.5 / 100;
	//const float rapportCyclique1ms = pleineEchelle * 8 / 100;
	//const float rapportCyclique2ms = pleineEchelle * 8.15 / 100;
	//const float rapportCycliqueAvMax = pleineEchelle * 8 / 100;
	//const float rapportCycliqueArMax = pleineEchelle * 6.5 / 100; //1065
	/*------------=========== Variables ==========------------*/
	//int _pwm = 0;  // how bright the LED
	int _buteeAvMax ;
	int _buteeArMax ;
	int _buteeAvMin ;
	int _propulsion = 0;
	int _previousPropulsion = 0;
	void _setPWM(int pwm);
	//float _calculPwm(int propulsion);
	void _recul(int pwm, bool freinage);
};

