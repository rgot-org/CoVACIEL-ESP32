
#include "CoVACIEL.h"


void CoVACIEL_CAN::updateRx() {

	parseData();
	//LOG_COVACIEL("_lastMessage:%d\n", _lastMessage);
	if (_lastMessage != 0)
	{
		if (_lastMessage & 1) {
			for (size_t i = 0; i < 4; i++)
			{
				if (CANMessages[PROPULSION].data[i] == 0xFF)
				{
					_propulsion = i;
					break;
				}
			}
			_vitesse = (CANMessages[PROPULSION].data[4] <<8 )| CANMessages[PROPULSION].data[5] ;
			LOG_COVACIEL("_vitesse:%d\n",_vitesse);
		}
		if (_lastMessage & 2)
		{
			for (size_t i = 0; i < 6; i++)
			{
				if (CANMessages[DIRECTION].data[i] == 0xFF)
				{
					_direction = i;
					break;
				}
			}
			_angle = int16_t((CANMessages[DIRECTION].data[4] <<8) | CANMessages[DIRECTION].data[5]) ;
			LOG_COVACIEL("direction\n");
		}
		if (_lastMessage & 4)
		{

			_distAvant = (CANMessages[AVANT].data[2] <<8)| CANMessages[AVANT].data[3];
			_distAvDroite45 = (CANMessages[AVANT].data[4] <<8)| CANMessages[AVANT].data[5];
			_distAvGauche45 = (CANMessages[AVANT].data[0] <<8)| CANMessages[AVANT].data[1];
			distance[DISTANCE_AVANT] = _distAvant;
			distance[DISTANCE_AV_DROITE_45] = _distAvDroite45;
			distance[DISTANCE_AV_GAUCHE_45] = _distAvGauche45;
			LOG_COVACIEL("dist av\n");
		}
		if (_lastMessage & 8)
		{
			_distAr = (CANMessages[ARRIERE].data[2] <<8)| CANMessages[ARRIERE].data[3];
			_distArDroite = (CANMessages[ARRIERE].data[4] <<8)| CANMessages[ARRIERE].data[5];
			_distArGauche = (CANMessages[ARRIERE].data[0] <<8)| CANMessages[ARRIERE].data[1];

			distance[DISTANCE_AR] = _distAr;
			distance[DISTANCE_AR_DROITE] = _distArDroite;
			distance[DISTANCE_AR_GAUCHE] = _distArGauche;
			LOG_COVACIEL("dist ar\n");
		}
		if (_lastMessage & 0x10)
		{
			_distAvant= (CANMessages[AVANT_EXT].data[2] << 8) | CANMessages[AVANT_EXT].data[3];
			_distAvDroite90 = (CANMessages[AVANT_EXT].data[4] <<8)| CANMessages[AVANT_EXT].data[5];
			_distAvGauche90 = (CANMessages[AVANT_EXT].data[0] <<8) | CANMessages[AVANT_EXT].data[1];
			distance[DISTANCE_AVANT] = _distAvant;
			distance[DISTANCE_AV_DROITE_90] = _distAvDroite90;
			distance[DISTANCE_AV_GAUCHE_90] = _distAvGauche90;
			LOG_COVACIEL("dist av\n");
		}
		_lastMessage = 0;
		_newMessage4Json = 1;


		
	}

}

void CoVACIEL_CAN::parseData()
{
	//LOG_COVACIEL("file:%d\n", inRxQueue());;
	if (inRxQueue() > 0)
	{
		_lastMessage = 0;
		LOG_COVACIEL("rx_queue:%d\n", inRxQueue());
		CanFrame msg;
		readFrame(msg, 100);
		switch (msg.identifier)
		{
		case CAN_ID_PROPULSION:
			_lastMessage = _lastMessage | 1;
			CANMessages[PROPULSION] = msg;
			break;
		case CAN_ID_DIRECTION:
			_lastMessage = _lastMessage | 2;
			CANMessages[DIRECTION] = msg;
			break;
		case CAN_ID_AVANT:
			_lastMessage = _lastMessage | 4;
			CANMessages[AVANT] = msg;
			break;
		case CAN_ID_ARRIERE:
			_lastMessage = _lastMessage | 8;
			CANMessages[ARRIERE] = msg;
			break;
		case CAN_ID_AVANT_EXT:
			_lastMessage = _lastMessage | 0x10;
			CANMessages[AVANT_EXT] = msg;
			break;
		default:
			_lastMessage = 0;
			break;
		}
		LOG_COVACIEL("_lastMessage:%d\n", _lastMessage);
	
	}
}

bool CoVACIEL_CAN::sendToCANBus(int can_id)
{
	bool test = false;
	CanFrame frame = { 0 };
	switch (can_id)
	{
	case CAN_ID_AVANT:
		frame.data[0] = _distAvGauche45 >> 8 & 0xFF;
		frame.data[1] = _distAvGauche45 & 0xFF;
		frame.data[2] = _distAvant >> 8 & 0xff;
		frame.data[3] = _distAvant & 0xff;
		frame.data[4] = _distAvDroite45 >> 8 & 0xff;
		frame.data[5] = _distAvDroite45 & 0xff;

		break;
	case CAN_ID_AVANT_EXT:
		frame.data[0] = _distAvGauche90 >> 8 & 0xff;
		frame.data[1] = _distAvGauche90 & 0xff;
		frame.data[2] = _distAvant >> 8 & 0xff;
		frame.data[3] = _distAvant & 0xff;
		frame.data[4] = _distAvDroite90 >> 8 & 0xff;
		frame.data[5] = _distAvDroite90 & 0xff;
		break;
	case CAN_ID_ARRIERE:
		frame.data[0] = _distArGauche >> 8 & 0xFF;
		frame.data[1] = _distArGauche & 0xFF;
		frame.data[2] = _distAr >> 8 & 0xff;
		frame.data[3] = _distAr & 0xff;
		frame.data[4] = _distArDroite >> 8 & 0xff;
		frame.data[5] = _distArDroite & 0xff;
		break;
	case CAN_ID_DIRECTION:
		frame.data[_direction] = 0xFF;
		frame.data[4] = _angle >> 8 & 0xff;
		frame.data[5] = _angle & 0xff;
		break;
	case CAN_ID_PROPULSION:
		
		if (_propulsion < 4)
		{
			frame.data[_propulsion] = 0xFF;
		}
		else if (_propulsion == 255) {
			frame.data[4] = _vitesse >> 8 & 0xff;
			frame.data[5] = _vitesse & 0xff;
		}
		LOG_COVACIEL("vit:%d\tprop:%d\n", _vitesse, _propulsion);
		break;
	default:
		break;
	}
	frame.identifier = can_id;
	frame.extd = 0;
	frame.data_length_code = 6;
	return writeFrame(frame, 10);
}


bool CoVACIEL_CAN::initialize(uint8_t rx, uint8_t tx)
{
	setPins(tx, rx);
	setRxQueueSize(5);
	setTxQueueSize(5);
	setSpeed(convertSpeed(250));
	return begin();
}

bool CoVACIEL_CAN::setPropulsion(byte commande)
{
	_propulsion = commande;
	return sendToCANBus(CAN_ID_PROPULSION);

}

bool CoVACIEL_CAN::setVitesse(int vitesse)
{
	_propulsion = 255;
	_vitesse = vitesse;
	return sendToCANBus(CAN_ID_PROPULSION);
}

int CoVACIEL_CAN::getPropulsion()
{
	return _propulsion;
}

String CoVACIEL_CAN::getPropulsionStr()
{
	String propulsion[] = { "Accelerer","Ralentir","Stop","Arriere" };
	return propulsion[_propulsion];
}
int CoVACIEL_CAN::getVitesse()
{
	return (int16_t)_vitesse;
}

bool CoVACIEL_CAN::setDirection(byte commande)
{
	_direction = commande;
	return sendToCANBus(CAN_ID_DIRECTION);
}

bool CoVACIEL_CAN::setAngleDirection(int angle)
{
	_angle = angle;
	return sendToCANBus(CAN_ID_DIRECTION);
}

bool CoVACIEL_CAN::setDistAv(uint16_t mm, bool send2canbus)
{
	_distAvant = mm;
	if (send2canbus)
	{
		return sendToCANBus(CAN_ID_AVANT);
	}
	return true;
}
bool CoVACIEL_CAN::setDistAvDroite45(uint16_t mm, bool send2canbus)
{
	_distAvDroite45 = mm;
	if (send2canbus)
	{
		return sendToCANBus(CAN_ID_AVANT);
	}
	return true;
}
bool CoVACIEL_CAN::setDistAvDroite90(uint16_t mm, bool send2canbus)
{
	_distAvDroite90 = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_AVANT_EXT);
	}
	return true;
}

bool CoVACIEL_CAN::setDistAvGauche45(uint16_t mm, bool send2canbus)
{
	_distAvGauche45 = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_AVANT);
	}
	return true;
}
bool CoVACIEL_CAN::setDistAvGauche90(uint16_t mm, bool send2canbus)
{
	_distAvGauche90 = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_AVANT_EXT);
	}
	return true;
}

bool CoVACIEL_CAN::setDistArDroite(uint16_t mm, bool send2canbus)
{
	_distArDroite = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_ARRIERE);
	}
	return true;
}

bool CoVACIEL_CAN::setDistArGauche(uint16_t mm, bool send2canbus)
{
	_distArGauche = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_ARRIERE);
	}
	return true;
}

bool CoVACIEL_CAN::updateTx(int canId)
{
	if (canId!=0)
	{
		return sendToCANBus(canId);
	}
	else
	{
		bool result = true;
		result=sendToCANBus(CAN_ID_ARRIERE);
		result &= sendToCANBus(CAN_ID_AVANT);
		result &= sendToCANBus(CAN_ID_AVANT_EXT);
		result &= sendToCANBus(CAN_ID_DIRECTION);
		result &= sendToCANBus(CAN_ID_PROPULSION);
		result &= sendToCANBus(CAN_ID_VITESSE);
		return result;
	}
}

bool CoVACIEL_CAN::setDistAr(uint16_t mm, bool send2canbus)
{
	_distAr = mm;
	if (send2canbus)
	{
	return sendToCANBus(CAN_ID_ARRIERE);
	}
	return true;
}

int CoVACIEL_CAN::getDirection()
{
	return _direction;
}

String CoVACIEL_CAN::getDirectionStr()
{
	String direction[] = { "Gauche","Droite","Tout droit" };
	return direction[_direction];
}

int CoVACIEL_CAN::getDistanceAvant()
{

	return _distAvant;
}

int CoVACIEL_CAN::getDistAvDroite45()
{
	return _distAvDroite45;
}

int CoVACIEL_CAN::getDistAvGauche45()
{
	return _distAvGauche45;
}

int CoVACIEL_CAN::getDistAvDroite90()
{
	return _distAvDroite90;
}

int CoVACIEL_CAN::getDistAvGauche90()
{
	return _distAvGauche90;
}

int CoVACIEL_CAN::getDistanceArriere()
{
	return _distAr;
}

int CoVACIEL_CAN::getDistanceArGauche()
{
	return _distArGauche;
}

int CoVACIEL_CAN::getDistanceArDroite()
{
	return _distArDroite;
}

int CoVACIEL_CAN::getDistance(byte secteur)
{
	if (secteur < NB_SECTEURS)
	{
		return distance[secteur];
	}
	return -1;
}

int CoVACIEL_CAN::getAngleDirection()
{
	return (int16_t) _angle;
}

bool CoVACIEL_CAN::setDistanceJson(String jsonDistance)
{
	JsonDocument doc;
	deserializeJson(doc, jsonDistance);
	if (!doc["distances"].isNull())
	{
		for (size_t i = 0; i < NB_SECTEURS; i++)
		{
			distance[i] = doc["distances"][i];
		}
	}
	_distAvant = distance[DISTANCE_AVANT];
	_distAvDroite45 = distance[DISTANCE_AV_DROITE_45];
	_distAvGauche45 = distance[DISTANCE_AV_GAUCHE_45];
	_distAr = distance[DISTANCE_AR];
	_distArDroite = distance[DISTANCE_AR_DROITE];
	_distArGauche = distance[DISTANCE_AR_GAUCHE];
	if (!sendToCANBus(CAN_ID_AVANT)) return false;
	if (!sendToCANBus(CAN_ID_ARRIERE)) return false;
	return true;


}

bool CoVACIEL_CAN::setDistance(int* distance,byte length)
{
	if (length>20)
	{
		_distAvant = distance[DISTANCE_AVANT];
		_distAvDroite45 = distance[DISTANCE_AV_DROITE_45];
		_distAvGauche45 = distance[DISTANCE_AV_GAUCHE_45];
		_distAvDroite90 = distance[DISTANCE_AV_DROITE_90];
		_distAvGauche90 = distance[DISTANCE_AV_GAUCHE_90];
		_distAr = distance[DISTANCE_AR];
		_distArDroite = distance[DISTANCE_AR_DROITE];
		_distArGauche = distance[DISTANCE_AR_GAUCHE];
		if (!sendToCANBus(CAN_ID_AVANT)) return false;
		if (!sendToCANBus(CAN_ID_AVANT_EXT)) return false;
		if (!sendToCANBus(CAN_ID_ARRIERE)) return false;
		return true;
	}
}


void CoVACIEL_CAN::canBus2SerialJson()
{
	parseData();
	updateRx();
	if (_newMessage4Json != 0)

	{
		JsonDocument document;
		JsonArray datas = document["payload"].to<JsonArray>();
		for (size_t i = 0; i < 5; i++)
		{
			if (CANMessages[i].identifier!=0)
			{
				JsonObject doc = datas.add<JsonObject>();
				doc["canId"] = CANMessages[i].identifier;
				JsonArray data = doc["data"].to<JsonArray>();
				switch (i)
				{
				case PROPULSION:
				case DIRECTION:
					for (size_t j = 0; j < CANMessages[i].data_length_code; j++)
					{
						data.add(CANMessages[i].data[j]);
					}
					break;
				case AVANT:
					data.add(_distAvGauche45);
					data.add(_distAvant);
					data.add(_distAvDroite45);
					break;
				case AVANT_EXT:
					data.add(_distAvGauche90);
					data.add(_distAvant);
					data.add(_distAvDroite90);
					break;
				case ARRIERE:
					data.add(_distArGauche);
					data.add(_distAr);
					data.add(_distArDroite);
					break;
				default:
					break;
				}

			}
		}
		serializeJson(document, Serial);
		Serial.println();
#if DEBUG
		_newMessage4Json = 0;
#endif // DEBUG


		
	}
}

String CoVACIEL_CAN::parseDistancesJson()
{
	updateRx();
	if (_lastMessage != -1) {
		String _str;
		JsonDocument doc;
		JsonArray data = doc["distances"].to<JsonArray>();
		for (size_t i = 0; i < 23; i++)
		{
			data.add(distance[i]);
		}
		serializeJson(doc, _str);
		return _str;
	}
}

String CoVACIEL_CAN::parseCanFrame2json()
{
	parseData();
	if (_lastMessage != 0)
	{
		JsonDocument document;
		JsonArray datas = document["payload"].to<JsonArray>();
		String _str;
		for (size_t i = 0; i < 4; i++)
		{
			int test = _lastMessage & (1 << i);
			LOG_COVACIEL("lastMsg:%d\ti:%d\ttest:%d\n", _lastMessage, i, test);
			if (test)
			{
				JsonObject doc = datas.add<JsonObject>();;
				doc["canId"] = CANMessages[i].identifier;
				JsonArray data = doc["data"].to<JsonArray>();
				for (size_t j = 0; j < CANMessages[i].data_length_code; j++)
				{
					data.add(CANMessages[i].data[j]);
				}
			}
		}
		serializeJson(document, _str);
		return _str;
	}
}



