#include "CoVACIEL.h"

Preferences CoVACIEL_prefs;
void CoVACIEL_propulsion::init(int pin)
{
	ledcAttachChannel(pin, SERVO_BASE_FREQ, RESOLUTION, PROPULSION_CHANNEL);
	ledcWriteChannel(PROPULSION_CHANNEL, arret);
	CoVACIEL_prefs.begin("confProp", true);
	_buteeArMax = CoVACIEL_prefs.getInt("buteeArMax",  -100);
	_buteeAvMax = CoVACIEL_prefs.getInt("bAvMax",  100);
	_buteeAvMin = CoVACIEL_prefs.getInt("bAvMin", rapportCycliqueArret);
	CoVACIEL_prefs.end();
	//rgbLedWrite(LED_PIN, 0, 20, 0);
	setPropulsion(0);
	delay(3000);
	//rgbLedWrite(LED_PIN, 0, 0, 0);
	

}

void CoVACIEL_propulsion::setPropulsion(int value)
{
	//int pwm = _calculPwm(value);
	_propulsion = value;
	if (value < 0)
	{
		(_previousPropulsion > 0) ? _recul(value, true) : _recul(value, false);

	}
	else
	{
		_setPWM(value);
	}
	_previousPropulsion = value;
}

void CoVACIEL_propulsion::_setPWM(int value)
{
	LOG_COVACIEL("pwmDesire:%d\n", value);
	int pwm = _buteeAvMin;
	if (value < 0)
	{
		pwm += -value * _buteeArMax / 100;
	}
	else
	{
		pwm += value * _buteeAvMax / 100;
	}
	LOG_COVACIEL("pwmCalcule=%d\tbuteeAR%d\tbuteeAv:%d\n", pwm,_buteeArMax,_buteeAvMax);
	int valeurContrainte = constrain(pwm, _buteeAvMin + _buteeArMax ,_buteeAvMin + _buteeAvMax);
	LOG_COVACIEL("pwmContraint=%d\n", valeurContrainte);
	ledcWriteChannel(PROPULSION_CHANNEL, valeurContrainte);
}
void CoVACIEL_propulsion::_recul(int pwm, bool freinage)
{
	if (freinage)
	{
		LOG_COVACIEL("pwmFrein=%d\n", _buteeAvMin - 408);
		ledcWriteChannel(PROPULSION_CHANNEL, _buteeAvMin-408);
		delay(300);
		_setPWM(0);
		delay(300);
	}
	LOG_COVACIEL("pwmFreinVoulue=%d\n", pwm);
	_setPWM(pwm);
}
void CoVACIEL_propulsion::setPWMAvMax(int value)
{
	//int pwm = _calculPwm(value);
	CoVACIEL_prefs.begin("confProp", false);
	CoVACIEL_prefs.putInt("bAvMax", value);
	CoVACIEL_prefs.end();
	_buteeAvMax = value;
	//ESP.restart();
}

void CoVACIEL_propulsion::setPWMAvMin(int value)
{
	//int pwm = _calculPwm(value);
	CoVACIEL_prefs.begin("confProp", false);
	CoVACIEL_prefs.putInt("bAvMin", value);
	CoVACIEL_prefs.end();
	_buteeAvMin = value;
	//ESP.restart();
}

void CoVACIEL_propulsion::setPWMArMax(int value)
{
	//int pwm = _calculPwm(value);
	CoVACIEL_prefs.begin("confProp", false);
	CoVACIEL_prefs.putInt("buteeArMax", value);
	CoVACIEL_prefs.end();
	_buteeArMax = value;
	//ESP.restart();
}
int CoVACIEL_propulsion::getPropulsion()
{
	return _propulsion;
}
int CoVACIEL_propulsion::getPWMAvMax()
{
	return _buteeAvMax;
}

int CoVACIEL_propulsion::getPWMArMax()
{
	return _buteeArMax;
}

int CoVACIEL_propulsion::getPWMAvMin()
{
	return _buteeAvMin;
}

void CoVACIEL_propulsion::reglageButees()
{
	{
		Serial.println(F("\n******************* REGLAGE DES BUTEES *********************"));
		Serial.println(F("* M -> PWM MAX, m -> vit. Ar MAX, 0 -> PWM 0       *"));
		Serial.println(F("* d pour diminuer la PWM MAX, D pour l'augmenter       *"));
		Serial.println(F("* r pour diminuer la PWM ar MAX, R  pour l'augmenter   *"));
		Serial.println(F("* s pour diminuer la PWM AV min, S pour l'augmenter         *"));
		Serial.println(F("************************************************************"));
		Serial.printf("\nActuellement: AvMax->%d\tArMax->%d\t arret->%d\n", _buteeAvMin+_buteeAvMax,_buteeAvMin+_buteeArMax,_buteeAvMin );
		//setPropulsion(0);
		//delay(3000);
		while (true)
		{
			if (Serial.available()) {
				String str = Serial.readString();
				if (str.startsWith("M")) // 
				{
					setPropulsion(100);
					Serial.printf("PMW AV MAX:\t%d\n", _buteeAvMin + _buteeAvMax);
				}
				else if (str.startsWith("m"))
				{
					setPropulsion(-100);
					Serial.printf("PWM AR MAX:\t%d\n", _buteeAvMin + _buteeArMax);
				}
				else if (str.startsWith("0")) {
					setPropulsion(0);
					Serial.printf("PWM STOP:\t%d\n", _buteeAvMin);
				}

				else if(str.startsWith("d"))
				{
					_buteeAvMax -= 1;
					setPWMAvMax(_buteeAvMax);
					setPropulsion(100);
					Serial.printf("PMW AV MAX:\t%d\n", _buteeAvMin + _buteeAvMax);
				}
				else if (str.startsWith("D"))
				{
					_buteeAvMax += 1;
					setPWMAvMax(_buteeAvMax);
					setPropulsion(100);
					Serial.printf("PWM AR MAX:\t%d\n", _buteeAvMin + _buteeAvMax);
				}
				else if (str.startsWith("r"))
				{
					_buteeArMax += 1;
					setPWMArMax(_buteeArMax);					
						setPropulsion(-100)	;
						Serial.printf("PWM AR MAX:\t%d\n", _buteeAvMin + _buteeArMax);
				}
				else if (str.startsWith("R"))
				{
					_buteeArMax -= 1;
					setPWMArMax(_buteeArMax);
					setPropulsion(-100);
					Serial.printf("PWM AR MAX:\t%d\n", _buteeAvMin + _buteeArMax);
				}
				else if (str.startsWith("s"))
				{
					_buteeAvMin -= 1;
					setPWMAvMin(_buteeAvMin);
					setPropulsion(0);
					Serial.printf("PMW STOP:\t%d\n", _buteeAvMin);
				}
				else if (str.startsWith("S"))
				{
					_buteeAvMin += 1;
					setPWMAvMin(_buteeAvMin);
					setPropulsion(0);
					Serial.printf("PMW STOP:\t%d\n", _buteeAvMin);
				}
				else if (str.startsWith("Q"))
				{
					Serial.println("Quitter !");
					break;
				}

			}
		}
	}
}

void CoVACIEL_propulsion::clearParams()
{
	CoVACIEL_prefs.begin("confProp", false);
	CoVACIEL_prefs.clear();
	CoVACIEL_prefs.end();
	ESP.restart();
}



//float CoVACIEL_propulsion::_calculPwm(int propulsion)
//{
//	int propContraint = constrain(propulsion, -51, 128);
//	LOG_COVACIEL("prop contraint:%d\n",propContraint);
//	return  pleineEchelle * (7.5 + 2.5 * propContraint / 128) / 100;
//}

//float CoVACIEL_propulsion::_setPwm(int propulsion)
//{
//	int propContraint = constrain(propulsion, -51, 128);
//	LOG_COVACIEL("prop contraint:%d\n",propContraint);
//	return  pleineEchelle * (7.5 + 2.5 * propContraint / 128) / 100;
//}


/********************************************************************************************
*                                         DIRECTION                                         *
*********************************************************************************************/
void CoVACIEL_direction::init(int pin)
{
	ledcAttachChannel(pin, SERVO_BASE_FREQ, RESOLUTION, DIRECTION_CHANNEL);
	ledcWriteChannel(DIRECTION_CHANNEL, rapportCycliqueCentre);
	CoVACIEL_prefs.begin("confDir", true);
	_buteeDroite = CoVACIEL_prefs.getInt("bd", rapportCycliqueMax);
	_buteeGauche = CoVACIEL_prefs.getInt("bg", rapportCycliqueMin);
	CoVACIEL_prefs.end();
	//rgbLedWrite(LED_PIN, 0, 20, 0);
}

void CoVACIEL_direction::setDirectionDegre(int value)
{
	int anglePwm = rapportCycliqueCentre + (_buteeDroite - _buteeGauche) * value / (2 * angleDegreMax);
	int valeurContrainte = constrain(anglePwm, _buteeGauche, _buteeDroite);
	ledcWriteChannel(DIRECTION_CHANNEL, valeurContrainte);
}



void CoVACIEL_direction::setButeeDroite(int value)
{
	CoVACIEL_prefs.begin("confDir", false);
	CoVACIEL_prefs.putInt("bd", value);
	CoVACIEL_prefs.end();
	_buteeDroite = value;
	_centre = (_buteeDroite - _buteeGauche) / 2;
}

void CoVACIEL_direction::setButeeGauche(int value)
{
	CoVACIEL_prefs.begin("confDir", false);
	CoVACIEL_prefs.putInt("bg", value);
	CoVACIEL_prefs.end();
	_buteeGauche = value;
	_centre = (_buteeDroite - _buteeGauche) / 2;
}

void CoVACIEL_direction::reglageButees()
{
	Serial.println(F("\n******************* REGLAGE DES BUTEES *********************"));
	Serial.println(F("* bg pour aller a la butee gauche, bd pour celle de droite *"));
	Serial.println(F("* g pour diminuer la butee gauche, G pour l'augmenter      *"));
	Serial.println(F("* d pour diminuer la butee droite, D pour l'augmenter      *"));
	Serial.println(F("************************************************************"));
	LOG_COVACIEL("\nActuellement: bd->%d\tbg->%d\n", getButeeDroite(), getButeeGauche());
	while (true)
	{
		if (Serial.available()) {
			String str = Serial.readString();
			if (str.startsWith("bg"))
			{
				setDirectionDegre(-18);
				LOG_COVACIEL("bg:\t%d\n",getButeeGauche());
			}
			else if (str.startsWith("bd"))
			{
				setDirectionDegre(18);
				LOG_COVACIEL("bd:\t%d\n", getButeeDroite());
			}
			else if (str.startsWith("g"))
			{
							_buteeGauche-=1;
				setButeeGauche(_buteeGauche);
				setDirectionDegre(-18);
				LOG_COVACIEL("bg:\t%d\n", getButeeGauche());
			}
			else if (str.startsWith("G"))
			{
				_buteeGauche += 1;
				setButeeGauche(_buteeGauche);
				setDirectionDegre(-18);
				LOG_COVACIEL("bg:\t%d\n", getButeeGauche());
			}
			else if (str.startsWith("d"))
			{
				_buteeDroite -= 1;
				setButeeDroite(_buteeDroite);
				setDirectionDegre(18);
				LOG_COVACIEL("bd:\t%d\n", getButeeDroite());
			}
			else if (str.startsWith("D"))
			{
				_buteeDroite += 1;
				setButeeDroite(_buteeDroite);
				setDirectionDegre(18);
				LOG_COVACIEL("bd:\t%d\n", getButeeDroite());
			}
			else if (str.startsWith("Q"))
			{
				Serial.println("Quitter !");
				break;
			}

		}
	}
}

int CoVACIEL_direction::getButeeDroite()
{
	return _buteeDroite;
}

int CoVACIEL_direction::getButeeGauche()
{
	return _buteeGauche;
}

void CoVACIEL_direction::clearParams()
{
	CoVACIEL_prefs.begin("confDir", false);
	CoVACIEL_prefs.clear();
	CoVACIEL_prefs.end();
	ESP.restart();
}
