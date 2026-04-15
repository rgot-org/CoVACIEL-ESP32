
#include "CoVACIEL.h"


void CoVACIEL_CAN::updateRx()
{
	parseData();
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
			_vitesse = (CANMessages[PROPULSION].data[4] << 8) | CANMessages[PROPULSION].data[5];
			LOG_COVACIEL("_vitesse:%d\n", _vitesse);
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
			_angle = int16_t((CANMessages[DIRECTION].data[4] << 8) | CANMessages[DIRECTION].data[5]);
			LOG_COVACIEL("direction\n");
		}
		if (_lastMessage & 4)
		{
			distance[DISTANCE_AVANT]        = (CANMessages[AVANT].data[2] << 8) | CANMessages[AVANT].data[3];
			distance[DISTANCE_AV_DROITE_45] = (CANMessages[AVANT].data[4] << 8) | CANMessages[AVANT].data[5];
			distance[DISTANCE_AV_GAUCHE_45] = (CANMessages[AVANT].data[0] << 8) | CANMessages[AVANT].data[1];
			LOG_COVACIEL("dist av\n");
		}
		if (_lastMessage & 8)
		{
			distance[DISTANCE_AR]        = (CANMessages[ARRIERE].data[2] << 8) | CANMessages[ARRIERE].data[3];
			distance[DISTANCE_AR_DROITE] = (CANMessages[ARRIERE].data[4] << 8) | CANMessages[ARRIERE].data[5];
			distance[DISTANCE_AR_GAUCHE] = (CANMessages[ARRIERE].data[0] << 8) | CANMessages[ARRIERE].data[1];
			LOG_COVACIEL("dist ar\n");
		}
		if (_lastMessage & 0x10)
		{
			distance[DISTANCE_AVANT]        = (CANMessages[AVANT_EXT].data[2] << 8) | CANMessages[AVANT_EXT].data[3];
			distance[DISTANCE_AV_DROITE_90] = (CANMessages[AVANT_EXT].data[4] << 8) | CANMessages[AVANT_EXT].data[5];
			distance[DISTANCE_AV_GAUCHE_90] = (CANMessages[AVANT_EXT].data[0] << 8) | CANMessages[AVANT_EXT].data[1];
			LOG_COVACIEL("dist av ext\n");
		}
		_lastMessage = 0;
		_newMessage4Json = 1;
	}
}

void CoVACIEL_CAN::parseData()
{
	// Vide toute la queue en accumulant les bits dans _lastMessage
	_lastMessage = 0;
	twai_status_info_t twai_status;
	while (twai_get_status_info(&twai_status) == ESP_OK && twai_status.msgs_to_rx > 0)
	{
		LOG_COVACIEL("rx_queue:%d\n", twai_status.msgs_to_rx);
		CanFrame msg;
		if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) break;
		switch (msg.identifier)
		{
		case CAN_ID_PROPULSION:
			_lastMessage |= 1;
			CANMessages[PROPULSION] = msg;
			break;
		case CAN_ID_DIRECTION:
			_lastMessage |= 2;
			CANMessages[DIRECTION] = msg;
			break;
		case CAN_ID_AVANT:
			_lastMessage |= 4;
			CANMessages[AVANT] = msg;
			break;
		case CAN_ID_ARRIERE:
			_lastMessage |= 8;
			CANMessages[ARRIERE] = msg;
			break;
		case CAN_ID_AVANT_EXT:
			_lastMessage |= 0x10;
			CANMessages[AVANT_EXT] = msg;
			break;
		case CAN_ID_ARRIERE_REQUEST:
			// requête reçue par la carte arrière → répondre immédiatement avec la mesure courante
			sendToCANBus(CAN_ID_ARRIERE);
			break;
		default:
			break;
		}
		LOG_COVACIEL("_lastMessage:%d\n", _lastMessage);
	}
}

bool CoVACIEL_CAN::sendToCANBus(int can_id)
{
	CanFrame frame = { 0 };
	switch (can_id)
	{
	case CAN_ID_AVANT:
		frame.data[0] = distance[DISTANCE_AV_GAUCHE_45] >> 8 & 0xFF;
		frame.data[1] = distance[DISTANCE_AV_GAUCHE_45] & 0xFF;
		frame.data[2] = distance[DISTANCE_AVANT] >> 8 & 0xFF;
		frame.data[3] = distance[DISTANCE_AVANT] & 0xFF;
		frame.data[4] = distance[DISTANCE_AV_DROITE_45] >> 8 & 0xFF;
		frame.data[5] = distance[DISTANCE_AV_DROITE_45] & 0xFF;
		break;
	case CAN_ID_AVANT_EXT:
		frame.data[0] = distance[DISTANCE_AV_GAUCHE_90] >> 8 & 0xFF;
		frame.data[1] = distance[DISTANCE_AV_GAUCHE_90] & 0xFF;
		frame.data[2] = distance[DISTANCE_AVANT] >> 8 & 0xFF;
		frame.data[3] = distance[DISTANCE_AVANT] & 0xFF;
		frame.data[4] = distance[DISTANCE_AV_DROITE_90] >> 8 & 0xFF;
		frame.data[5] = distance[DISTANCE_AV_DROITE_90] & 0xFF;
		break;
	case CAN_ID_ARRIERE:
		frame.data[0] = distance[DISTANCE_AR_GAUCHE] >> 8 & 0xFF;
		frame.data[1] = distance[DISTANCE_AR_GAUCHE] & 0xFF;
		frame.data[2] = distance[DISTANCE_AR] >> 8 & 0xFF;
		frame.data[3] = distance[DISTANCE_AR] & 0xFF;
		frame.data[4] = distance[DISTANCE_AR_DROITE] >> 8 & 0xFF;
		frame.data[5] = distance[DISTANCE_AR_DROITE] & 0xFF;
		break;
	case CAN_ID_DIRECTION:
		frame.data[_direction] = 0xFF;
		frame.data[4] = _angle >> 8 & 0xFF;
		frame.data[5] = _angle & 0xFF;
		break;
	case CAN_ID_PROPULSION:
		if (_propulsion < 4)
		{
			frame.data[_propulsion] = 0xFF;
		}
		else if (_propulsion == 255)
		{
			frame.data[4] = _vitesse >> 8 & 0xFF;
			frame.data[5] = _vitesse & 0xFF;
		}
		LOG_COVACIEL("vit:%d\tprop:%d\n", _vitesse, _propulsion);
		break;
	default:
		break;
	}
	frame.identifier = can_id;
	frame.extd = 0;
	frame.data_length_code = 6;
	return twai_transmit(&frame, pdMS_TO_TICKS(10)) == ESP_OK;
}


bool CoVACIEL_CAN::init(uint8_t rx, uint8_t tx)
{
	twai_general_config_t g_config = {
		.mode           = TWAI_MODE_NORMAL,
		.tx_io          = (gpio_num_t)tx,
		.rx_io          = (gpio_num_t)rx,
		.clkout_io      = TWAI_IO_UNUSED,
		.bus_off_io     = TWAI_IO_UNUSED,
		.tx_queue_len   = 5,
		.rx_queue_len   = 5,
		.alerts_enabled = TWAI_ALERT_NONE,
		.clkout_divider = 0,
		.intr_flags     = ESP_INTR_FLAG_LEVEL1
	};
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
	return twai_start() == ESP_OK;
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
	static const char* propulsion[] = { "Accelerer", "Ralentir", "Stop", "Arriere" };
	if (_propulsion < 4) return propulsion[_propulsion];
	return "";
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
	distance[DISTANCE_AVANT] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_AVANT);
	return true;
}

bool CoVACIEL_CAN::setDistAvDroite45(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AV_DROITE_45] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_AVANT);
	return true;
}

bool CoVACIEL_CAN::setDistAvDroite90(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AV_DROITE_90] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_AVANT_EXT);
	return true;
}

bool CoVACIEL_CAN::setDistAvGauche45(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AV_GAUCHE_45] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_AVANT);
	return true;
}

bool CoVACIEL_CAN::setDistAvGauche90(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AV_GAUCHE_90] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_AVANT_EXT);
	return true;
}

bool CoVACIEL_CAN::setDistArDroite(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AR_DROITE] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_ARRIERE);
	return true;
}

bool CoVACIEL_CAN::setDistArGauche(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AR_GAUCHE] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_ARRIERE);
	return true;
}

bool CoVACIEL_CAN::updateTx(int canId)
{
	if (canId != 0)
	{
		return sendToCANBus(canId);
	}
	else
	{
		bool result = true;
		result  = sendToCANBus(CAN_ID_ARRIERE);
		result &= sendToCANBus(CAN_ID_AVANT);
		result &= sendToCANBus(CAN_ID_AVANT_EXT);
		result &= sendToCANBus(CAN_ID_DIRECTION);
		result &= sendToCANBus(CAN_ID_PROPULSION);
		return result;
	}
}

bool CoVACIEL_CAN::setDistAr(uint16_t mm, bool send2canbus)
{
	distance[DISTANCE_AR] = mm;
	if (send2canbus) return sendToCANBus(CAN_ID_ARRIERE);
	return true;
}

int CoVACIEL_CAN::getDirection()
{
	return _direction;
}

String CoVACIEL_CAN::getDirectionStr()
{
	static const char* direction[] = { "Gauche", "Droite", "Tout droit" };
	if (_direction < 3) return direction[_direction];
	return "";
}

int CoVACIEL_CAN::getDistanceAvant()
{
	return distance[DISTANCE_AVANT];
}

int CoVACIEL_CAN::getDistAvDroite45()
{
	return distance[DISTANCE_AV_DROITE_45];
}

int CoVACIEL_CAN::getDistAvGauche45()
{
	return distance[DISTANCE_AV_GAUCHE_45];
}

int CoVACIEL_CAN::getDistAvDroite90()
{
	return distance[DISTANCE_AV_DROITE_90];
}

int CoVACIEL_CAN::getDistAvGauche90()
{
	return distance[DISTANCE_AV_GAUCHE_90];
}

int CoVACIEL_CAN::getDistanceArriere()
{
	CanFrame frame = { 0 };
	frame.identifier = CAN_ID_ARRIERE_REQUEST;
	frame.extd = 0;
	frame.data_length_code = 0; // trame vide = requête
	twai_transmit(&frame, pdMS_TO_TICKS(10));
	return distance[DISTANCE_AR]; // retourne la dernière valeur connue, la réponse sera reçue via updateRx()
}

int CoVACIEL_CAN::getDistanceArGauche()
{
	return distance[DISTANCE_AR_GAUCHE];
}

int CoVACIEL_CAN::getDistanceArDroite()
{
	return distance[DISTANCE_AR_DROITE];
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
	return (int16_t)_angle;
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
	if (!sendToCANBus(CAN_ID_AVANT)) return false;
	if (!sendToCANBus(CAN_ID_ARRIERE)) return false;
	return true;
}

bool CoVACIEL_CAN::setDistance(int* distances, byte length)
{
	if (length >= NB_SECTEURS)
	{
		for (size_t i = 0; i < NB_SECTEURS; i++)
			distance[i] = distances[i];
		if (!sendToCANBus(CAN_ID_AVANT)) return false;
		if (!sendToCANBus(CAN_ID_AVANT_EXT)) return false;
		if (!sendToCANBus(CAN_ID_ARRIERE)) return false;
		return true;
	}
	return false;
}


void CoVACIEL_CAN::canBus2SerialJson()
{
	updateRx(); // parseData() est appelé par updateRx(), pas besoin de l'appeler en double
	if (_newMessage4Json != 0)
	{
		JsonDocument document;
		JsonArray datas = document["payload"].to<JsonArray>();
		for (size_t i = 0; i < 5; i++)
		{
			if (CANMessages[i].identifier != 0)
			{
				JsonObject doc = datas.add<JsonObject>();
				doc["canId"] = CANMessages[i].identifier;
				JsonArray data = doc["data"].to<JsonArray>();
				switch (i)
				{
				case PROPULSION:
				case DIRECTION:
					for (size_t j = 0; j < CANMessages[i].data_length_code; j++)
						data.add(CANMessages[i].data[j]);
					break;
				case AVANT:
					data.add(distance[DISTANCE_AV_GAUCHE_45]);
					data.add(distance[DISTANCE_AVANT]);
					data.add(distance[DISTANCE_AV_DROITE_45]);
					break;
				case AVANT_EXT:
					data.add(distance[DISTANCE_AV_GAUCHE_90]);
					data.add(distance[DISTANCE_AVANT]);
					data.add(distance[DISTANCE_AV_DROITE_90]);
					break;
				case ARRIERE:
					data.add(distance[DISTANCE_AR_GAUCHE]);
					data.add(distance[DISTANCE_AR]);
					data.add(distance[DISTANCE_AR_DROITE]);
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
#endif
	}
}

String CoVACIEL_CAN::parseDistancesJson()
{
	updateRx();
	if (_lastMessage != 0) {
		String _str;
		JsonDocument doc;
		JsonArray data = doc["distances"].to<JsonArray>();
		for (size_t i = 0; i < NB_SECTEURS; i++)
			data.add(distance[i]);
		serializeJson(doc, _str);
		return _str;
	}
	return "";
}

String CoVACIEL_CAN::parseCanFrame2json()
{
	parseData();
	if (_lastMessage != 0)
	{
		JsonDocument document;
		JsonArray datas = document["payload"].to<JsonArray>();
		String _str;
		for (size_t i = 0; i < 5; i++) // 5 messages : PROPULSION, DIRECTION, AVANT, ARRIERE, AVANT_EXT
		{
			int test = _lastMessage & (1 << i);
			LOG_COVACIEL("lastMsg:%d\ti:%d\ttest:%d\n", _lastMessage, i, test);
			if (test)
			{
				JsonObject doc = datas.add<JsonObject>();
				doc["canId"] = CANMessages[i].identifier;
				JsonArray data = doc["data"].to<JsonArray>();
				for (size_t j = 0; j < CANMessages[i].data_length_code; j++)
					data.add(CANMessages[i].data[j]);
			}
		}
		serializeJson(document, _str);
		return _str;
	}
	return "";
}
