#pragma once

#include "Arduino.h"
#include <functional> // Pour std::function et std::bind
//#define lidarDebugSerial lidarConsole
#ifndef lidarConsole
#define lidarConsole Serial
//#define lidarConsole telnet
#endif // !lidarConsole


struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
  const uint8_t pin;
  const uint32_t freq; //Hz
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)

  uint16_t targetRPM = 0;
  uint16_t Kp = 150;
  uint16_t lastRPM = 0;
  //uint32_t RPMcontrolTimer;
  //const uint32_t RPMcontrolInterval = 5000;
  const uint32_t angleToRPMmult = (1000000*60)/(360<<6); // 60 million microseconds per minute, 360deg per rotation, lidar angle data comes in 'q6' format (divide by 64 to get degrees)
  
  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init() {
	  
    //ledcSetup(channel, freq, 8);
    //ledcAttachPin(pin, channel);
	ledcAttachChannel(pin, freq, 8, channel);

    setPWM(60);
  }
  inline void setPWM(uint8_t newPWMval) {
	  ledcWrite(channel, activeHigh ? newPWMval*255/100 : (255-newPWMval)*255/100);
  }
  uint16_t RPM_PID(uint32_t angleDiff, uint32_t microsDiff) {
    uint16_t currentRPM = (angleDiff*angleToRPMmult)/microsDiff; //calculate RPM in an integer friendly way (because it's fast)
    //uint32_t newPWMval = Kp * targetRPM;
    //newPWMval = constrain(newPWMval>>8, 0, 255); // divide by 256 and constrain
    //setPWM((uint8_t)newPWMval);
    lastRPM = currentRPM;
    return(currentRPM);
  }
};

const uint8_t CMD_START_FLAG = 0xA5;
const uint8_t RESP_DESCR_START_FLAGS[2] = { 0xA5, 0x5A };
const uint8_t CMD_STOP = 0x25; //stop the lidar (no payload, no repsonse (wait at least 1ms))
const uint8_t CMD_RESET = 0x40; //reset the lidar core (no payload, no repsonse (wait at least 2ms)) (used for exiting "Protection Stop state")
const uint8_t CMD_SCAN = 0x20; //start scanning (once rotation speed is good and stable) (no payload, multiple response) (this is more of a legacy scan mode, the real good stuff is in EXPRESS_SCAN, i think)
const uint8_t CMD_EXPRESS_SCAN = 0x82; //start scanning at highest speed (idk if that's sample rate or rotation speed) (once rotation speed is good???) (has payload, multiple response) (requires firmware >= 1.17)
const uint8_t CMD_FORCE_SCAN = 0x21; //start scanning (regardless of rotation speed) (no payload, multiple response)
const uint8_t CMD_GET_INFO = 0x50; //get device info (no payload, single response)
const uint8_t CMD_GET_HEALTH = 0x52; //get device health info (no payload, single response)
const uint8_t CMD_GET_SAMPLERATE = 0x59; //get samplerate (no payload, single response) (requires firmware >= 1.17)
const uint8_t CMD_GET_LIDAR_CONF = 0x84; //request LIDAR configuration (lots of info about scan modes) (has payload, single response) (requires firmware >= 1.24)

// response descriptor codes:
const uint8_t RESP_DESCR_SENDMODE_SINGLE_RESPONSE = 0b00000000; // single request - single response
const uint8_t RESP_DESCR_SENDMODE_MULTI_RESPONSE = 0b01000000; // single request - multiple response
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_LEGACY = 0x82;
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_EXTEND = 0x84;
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_DENSE = 0x85;

const uint8_t GET_HEALTH_STATUS_GOOD = 0; // (status byte of GET_HEALTH response)
const uint8_t GET_HEALTH_STATUS_WARNING = 1;
const uint8_t GET_HEALTH_STATUS_ERROR = 2; // 'protection stop state'

//GET_LIDAR_CONF specific codes:
const uint8_t GET_CONF_SCAN_MODE_COUNT = 0x70; // (no payload, 16bit response)
const uint8_t GET_CONF_SCAN_MODE_SAMPLETIME = 0x71; // (16bit payload, 32bit response) (response/256.0 = response>>8 = microseconds)
const uint8_t GET_CONF_SCAN_MODE_MAX_DIST = 0x74; // (16bit payload, 32bit response) (response/256.0 = response>>8 = microseconds)
const uint8_t GET_CONF_SCAN_MODE_ANS_TYPE = 0x75; // (16bit payload, 8bit response)
const uint8_t GET_CONF_SCAN_MODE_NAME = 0x7F; // (16bit payload, String response)
const uint8_t GET_CONF_SCAN_MODE_TYPICAL = 0x7C; // (no payload, 16bit response)

const uint8_t CONF_SCAN_MODE_ANS_TYPE_STANDARD = 0x81; // standard mode (presumably what i called struct lidarStandardData {} )
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS = 0x82; // express mode (legacy i think)
const uint8_t CONF_SCAN_MODE_ANS_TYPE_ULTRA_CAP = 0x83; // 'ultra capsulated' mode, used for 'boost, stability and sensitivity mode'... i'm pretty sure my A1M8 can't do those anyway
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND = 0x84; // express mode, extended version? (sometimes also called 'ultra capsulated'?)
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS_DENSE = 0x85; // express mode, dense version

//response description 'dataType'
const uint8_t RESP_TYPE_SCAN = 0x81; // also for FORCE_SCAN
const uint8_t RESP_TYPE_SCAN_EXPRESS_LEGACY = 0x82;
//const uint8_t RESP_TYPE_SCAN_HQ = 0x83; // taken from SDK code, but i can't find any real mention of 'HQ' mode (and the typedefs in the SDK code are unfamiliar
const uint8_t RESP_TYPE_SCAN_EXPRESS_EXTEND = 0x84; // a.k.a. 'ultra capsulated'
const uint8_t RESP_TYPE_SCAN_EXPRESS_DENSE = 0x85;
const uint8_t RESP_TYPE_GET_INFO = 0x04;
const uint8_t RESP_TYPE_GET_HEALTH = 0x06;
const uint8_t RESP_TYPE_GET_SAMPLERATE = 0x15;
const uint8_t RESP_TYPE_GET_LIDAR_CONF = 0x20;
const uint8_t RESP_TYPE_INVALID = 0x00; // not from the official documentation, this is soemthing i came up with the let me return objects even when the read fails (and identify 

//EXPRESS WORK MODES (to be put into EXPRESS_SCAN request payload (first byte))
const uint8_t EXPRESS_SCAN_WORKING_MODE_LEGACY = 0; //working mode for legacy mode, exists on all lidar models
const uint8_t EXPRESS_SCAN_WORKING_MODE_BOOST = 2; //see what comes up when you run GET_LIDAR_CONF
const uint8_t EXPRESS_SCAN_WORKING_MODE_SENSITIVITY = 3; //these values appear to be indices of scan modes
const uint8_t EXPRESS_SCAN_WORKING_MODE_STABILITY = 4; //the documentation regarding these is somewhat lacking

//bit masks for isolating various tangled data in packets (not used in all cases, so do check the data structs manually if you change anything here)
const uint8_t RESP_DESCR_SENDMODE_BITS = 0b11000000; //the responseLength is 30 bits long, the last 2 bits indicate sendMode
const uint8_t STANDARD_DATA_ROT_START_BITS = 0b00000011; //the lowest 2 bits of the first byte is the rotation_start flag(s)
const uint8_t STANDARD_DATA_CHECK_BIT = 0b00000001; //the LSBit of the second byte is a check but, should always be 1
const uint8_t EXPRESS_DATA_LEGACY_SYNC_BITS = 0xF0; //the MSB half of the first 2 legacy express measurement packet bytes are the sync thingies
const uint8_t EXPRESS_DATA_LEGACY_ROT_START_BIT = 0b10000000; //the MSBit of the 4th byte of the legacy express measurement packet is the rotation_start flag
const uint8_t EXPRESS_DATA_LEGACY_DIST_BITS = 0b11111100; //fuck it, just see page 22 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
const uint16_t EXPRESS_DATA_EXTEND_MAJOR_BITS = 0x0FFF; //first 2 bits of the 4-byte 'ultra_cabin' (page 26)

// for encoding requests to the lidar:
struct lidarCMD { // less of a struct and more of a wrapper around a single byte array, but whatever
	uint8_t* _data; // a (variable size) byte buffer for the data to be sent (initialized with malloc() on initialization, freed in destructor)
	lidarCMD(uint8_t CMD, uint8_t payloadSize) : _data((uint8_t*)calloc(payloadSize + 4, 1)) { _data[0] = CMD_START_FLAG; _data[1] = CMD; _data[2] = payloadSize; }
	lidarCMD(uint8_t* _dataInput) { _data = _dataInput; } // not recommended (but at least you dont NEED to make a second buffer if you've (foolishly) chose to make your own byte buffer)
	~lidarCMD() { delete _data; } //my constructor is too funky for the compiler to think of this. (make sure to check for memory leaks BTW...)
	inline uint8_t*& operator&() { return(_data); } //any attempts to retrieve the address of this object should be met with a (reference to) _data (it's the same address, just an implicit typecast)
	inline uint8_t& operator[](size_t index) { return(_data[index + 3]); } // the payload starts after the StartFlag, CMDbyte and payloadSize bytes.
	inline uint8_t& CMD() { return(_data[1]); }
	inline uint8_t& payloadSize() { return(_data[2]); }
	//inline uint8_t& checksum() {return(_data[_data[2]+3]);}
	inline uint32_t& payloadAsUint32_t(uint8_t index = 0) { return((uint32_t&)_data[3 + index]); } // used for GET_LIDAR_CONF 'type' value (fill with values like GET_CONF_SCAN_MODE_COUNT
	inline uint16_t& payloadAsUint16_t(uint8_t index = 0) { return((uint16_t&)_data[3 + index]); } // alternatively, you could do " uint16_t& temp = (uint16_t&)cmdObj[index]; temp = blabla; "
	uint8_t calcChecksum() {
		uint8_t checksumVal = 0;
		for (uint8_t i = 0; i < (_data[2] + 3); i++) { checksumVal ^= _data[i]; } // '^' is the XOR operator
		return(checksumVal);
	}
	inline uint16_t sendSize() { return(_data[2] ? (_data[2] + 4) : 2); } //the size of _data is either payloadSize+4 {start,CMD,size,payload[],checksum} or 3 {start,CMD,size}, but if there is no payload, only SEND 2.
	uint8_t* send() {
		if (_data[2]) { _data[_data[2] + 3] = calcChecksum(); } //if there is a payload, calculate and store checksum as the last byte of _data
		return(_data);
	}
};

template<uint8_t arraySize> struct byteArrayStruct {
	uint8_t _data[arraySize];
	//  byteArrayStruct() {} // default initializer
	//  byteArrayStruct(uint8_t* _dataInput) {for(uint8_t i=0;i<sizeof(_data);i++){_data[i]=_dataInput[i];}}
	//  byteArrayStruct(uint8_t* _dataInput) { _data = _dataInput; }
	inline uint8_t* operator&() { return(_data); } //any attempts to retrieve the address of this object should be met with a byte pointer to _data (it's the same address, just an implicit typecast)
	inline uint8_t& operator[](size_t index) { return(_data[index]); } //i'm hoping the compiler will just know what i mean
}; // that saves a few lines through inheritance

// for decoding response descriptors from the lidar:
struct lidarRespDescr : public byteArrayStruct<5> { // less of a struct and more of a wrapper around a single byte array, but whatever
	uint32_t responseLength() {
		uint32_t responseLength;  uint8_t* bytePointer = (uint8_t*)&responseLength;
		memcpy(bytePointer, _data, sizeof(responseLength));
		bytePointer[sizeof(responseLength) - 1] &= ~RESP_DESCR_SENDMODE_BITS;
		return(responseLength);
	}
	inline uint8_t sendMode() { return(_data[3] & RESP_DESCR_SENDMODE_BITS); }
	inline uint8_t& dataType() { return(_data[4]); } //???
};

// for decoding all of the different measurement response packet formats from the lidar:
struct lidarStandardData : public byteArrayStruct<5> {
	inline uint8_t quality() { return((_data[0] & (~STANDARD_DATA_ROT_START_BITS)) >> 2); } // (6bit number) quality of measurement ("reflected laser pulse strength")
	inline uint8_t rotStartFlag() { return(_data[0] & STANDARD_DATA_ROT_START_BITS); } // 1 when new rotation starts, 2 otherwise (if it's 0 or 3 then something is very wrong!)
	inline bool checkBit() { return(_data[1] & STANDARD_DATA_CHECK_BIT); } //should always be 1
	inline uint16_t angle() { uint16_t angle = (_data[2] << 7); return(angle | (_data[1] >> 1)); } //(15bit number) divide by 64 (bitshift by 6 (or multiply by 0.015625) if you want to be quick) to get degrees
	inline uint16_t& dist() { return((uint16_t&)_data[3]); } // divide by 4 (bitshift by 2 (or multiply by 0.25) if you want to be quick) to get millimeters
};

template<uint8_t bufferSize> struct _lidarExpressDataBase { //this is the parts all Express data packets have in common
	uint8_t _data[bufferSize]; // unfortunately, this template class can't seem to work with byteArrayStruct, so here are those functions again
	inline uint8_t* operator&() { return(_data); } //any attempts to retrieve the address of this object should be met with a (byte pointer) to _data (it's the same address, just an implicit typecast)
	inline uint8_t& operator[](size_t index) { return(_data[index]); } //i'm hoping the compiler will just know what i mean
	inline uint8_t sync() { return((_data[0] & EXPRESS_DATA_LEGACY_SYNC_BITS) | ((_data[1] & EXPRESS_DATA_LEGACY_SYNC_BITS) >> 4)); } // sync 'byte' (two half bytes) should equal 0xA5 when put together like this
	inline uint8_t checksum() { return((_data[0] & (~EXPRESS_DATA_LEGACY_SYNC_BITS)) | ((_data[1] & (~EXPRESS_DATA_LEGACY_SYNC_BITS)) << 4)); } // checksum byte
	inline uint16_t startAngle() { uint16_t startAngle = (_data[3] & (~EXPRESS_DATA_LEGACY_ROT_START_BIT)); return((startAngle << 8) | _data[2]); } //(15bit number) divide by 64 (bitshift by 6 (or multiply by 0.015625) if you want to be quick) to get degrees
	inline bool rotStartFlag() { return(_data[3] & EXPRESS_DATA_LEGACY_ROT_START_BIT); }
	uint8_t calcChecksum() { //seems to work?
		uint8_t checksumVal = 0; //= 0xA0 ^ 0x50 = 0xF0   if the first 2 bytes are included in the checksum
		for (uint8_t i = 2; i < bufferSize; i++) { checksumVal ^= _data[i]; } // '^' is the XOR operator
		return(checksumVal);
	}
	//the dist and angle extraction functions depend on the different types of packets. This struct is inherited into those
};

struct lidarExpressDataLegacy : public _lidarExpressDataBase<84> {
	inline uint16_t dist(uint8_t index) { // a 14bit number
		uint8_t offIndex = 4 + ((index / 2) * 5) + ((index % 2) * 2); //index runs from 0 to 31, offIndex runs from 4 to 81
		uint16_t dist = (_data[offIndex] & EXPRESS_DATA_LEGACY_DIST_BITS);
		return((_data[offIndex + 1] << 6) | (dist >> 2)); // return in direct millimeters (becuase that is the actual resolution of this packet type)
		//return((_data[offIndex+1]<<8)|dist); // return in 'q2' millimeters (mm*4), to conform with the standard data packets?
	} // express legacy distance measurements have only 14bit resolution, so they are in mm directly (a max value of 2^14= 16384mm = 16.384m >= the A1M8's 12M range)
	inline int8_t deltaAngle(uint8_t index) { //note: deltaAngle is a SIGNED 6bit number in 'q3' format (divide by (1<<3) or bitshift by 3 to get degrees) (from the formula on page 31 it seems like this number maybe indicate deviation from the expected angle progression)
		uint8_t offIndex = 4 + ((index / 2) * 5); // just see page 22 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
		//    int8_t deltaAngle = (_data[offIndex+((index%2)*2)] & 0b00000001)<<4; //the last bit of the first/third byte is the 5th databit in deltaAngle
		//    deltaAngle |= ((_data[offIndex+((index%2)*2)] & 0b00000010) ? 0b10000000 : 0); //the second bit of the first/third byte is the sign bit of deltaAngle????
		int8_t deltaAngle = (_data[offIndex + ((index % 2) * 2)] & (~EXPRESS_DATA_LEGACY_DIST_BITS)) << 6; deltaAngle = deltaAngle >> 2; //bitshift all the way left, and then back again to right right to use signed bitshifting (fills in based on sign bit)
		if (index % 2) { //if it's an odd numbered index
			return(deltaAngle | (_data[offIndex + 4] >> 4)); //i'm hoping it will fill the byte with 0th when shifting
		}
		else {
			return(deltaAngle | (_data[offIndex + 4] & 0x0F));
		}
	} // "in the format of q3 in the unit degrees" i dont really know
};

struct lidarExpressDataExtended : public _lidarExpressDataBase<132> {
	inline uint32_t& wholeCabin(uint8_t cabinIndex = 0) { return((uint32_t&)_data[4 + cabinIndex * 4]); }
	inline uint16_t major(uint8_t cabinIndex) { //get the 'major' value of each 'ultra_cabin'. see page 26 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
		uint16_t& firstTwoBytes = (uint16_t&)_data[4 + cabinIndex * 4]; //cabinIndex runs from 0 to 31
		return(firstTwoBytes & EXPRESS_DATA_EXTEND_MAJOR_BITS);
	}
	inline int32_t predictOne(uint8_t cabinIndex) { //get the 'predict1' value of each 'ultra_cabin'
		int32_t& combined_x3 = (int32_t&)_data[4 + cabinIndex * 4]; //cabinIndex runs from 0 to 31
		return((combined_x3 << 10) >> 22); // shift left and right to make use of signed shifting (pads left bits based on sign bit)
	}
	inline int32_t predictTwo(uint8_t cabinIndex) { //get the 'predict1' value of each 'ultra_cabin'
		int32_t& combined_x3 = (int32_t&)_data[4 + cabinIndex * 4]; //cabinIndex runs from 0 to 31
		return((combined_x3) >> 22); // shift right to make use of signed shifting (pads left bits based on sign bit)
	}
};

struct lidarExpressDataDense : public _lidarExpressDataBase<84> { //the simplest format, but doesn't include Intermediary angles (you just have to calculate thise as (index/40)*(start_angle - last_start_angle))
	inline uint16_t& dist(uint8_t index) { return((uint16_t&)_data[4 + (index * 2)]); } // "unit is millimeters" so the max distance measured is 65meters??? page 12 of the protocol manual does state that the S1 lidars can do 40m in this mode...
};


// for decoding the information request response packets from the lidars
struct lidarGetInfoResponse : public byteArrayStruct<20> {
	inline uint8_t& model() { return(_data[0]); } //the first byte contains the model data
	inline uint8_t& firmwareMinor() { return(_data[1]); } //part after the decimal point
	inline uint8_t& firmwareMajor() { return(_data[2]); } //part before the decimal point
	inline uint8_t& hardware() { return(_data[3]); } //the fourth byte contains the model data
	inline uint8_t* serialNumber() { return(_data + 4); } // serialNumber is a 16-byte array (128bit number) (this returns a pointer to it)
	inline uint8_t& serialNumber(uint8_t index) { return(_data[index + 4]); } // serialNumber is a 16-byte array (128bit number) (hopefully the compiler makes efficient use of this)
};

struct lidarGetHealthResponse : public byteArrayStruct<3> {
	inline uint8_t& status() { return(_data[0]); } //the first byte contains the status data (0=good, 1=warning, 2=error)
	inline uint16_t& errorCode() { return((uint16_t&)_data[1]); } //the second and third bytes make the error code
};

struct lidarGetSamplerateResponse : public byteArrayStruct<4> {
	inline uint16_t& standard() { return((uint16_t&)_data[0]); } //the standard sample time (for using CMD_SCAN or CMD_FORCE_SCAN) in microseconds
	inline uint16_t& express() { return((uint16_t&)_data[2]); } //the express sample time (for using CMD_EXPRESS_SCAN) in microseconds
};

struct lidarGetLidarConfResponse { // i would have liked to set this at a constant size as well, but it must also be able to carry a String (otherwise i'd just spec it for the largest possible content)
	uint8_t* _data; // a (variable size) byte buffer for the data to be sent (initialized with malloc() on initialization, freed in destructor)
	lidarGetLidarConfResponse(uint8_t payloadSize) : _data((uint8_t*)malloc(payloadSize + sizeof(uint32_t))) {} //room for the payload and the (4byte) 'type' value
	~lidarGetLidarConfResponse() { delete _data; } //my constructor is too funky for the compiler to think of this.
	inline uint8_t*& operator&() { return(_data); } //any attempts to retrieve the address of this object should be met with a (reference to) _data (it's the same address, just an implicit typecast)
	inline uint8_t& operator[](size_t index) { return(_data[index + 4]); } // the payload starts after typeVal
	inline uint32_t& typeVal() { return((uint32_t&)_data[0]); } //GET_LIDAR_CONF returns a 'type' variable, although what this may contain is not entirely clear (it may be an echo of the request you sent it, idk)
	inline operator uint16_t& () { return((uint16_t&)_data[4]); } //cast to unsigned 16bit integer
	inline operator uint32_t& () { return((uint32_t&)_data[4]); } //cast to unsigned 32bit integer
	inline operator uint8_t& () { return(_data[4]); } //cast to unsigned 8bit integer
	//inline String operator() { TBD } //cant be done because this struct does NOT keep track of its own size (so it wouldn't know how long to make the string)
	inline char* charArray() { return((char*)_data + 4); } // (really just a pointer to the real payload)
	inline char& charArray(uint8_t index) { return((char&)_data[index + 4]); } // the String payload may be easier to read as a char array
};



class RPlidar {
public:
	HardwareSerial& lidarSerial;
	std::function<void(uint16_t, uint16_t, uint8_t, int8_t)> postParseCallback;
	uint8_t scanResponseFormat = 0; // 0 if scanning has not been requested (correctly), RESP_TYPE_SCAN or similar if scanning has been requested
	//void (*postParseCallback)(uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) = NULL;
	const uint32_t slowSerialTimout = 10; // packets like lidarRespDescr may take a while to arrive
	const uint32_t fastSerialTimout = 0; // scan data packets should be quick to read out

	int16_t expressDataAngleDelta; // used to calculate intermediate angles (only needed for express packages)
	uint16_t expressDataLastStartAngle; // used to calculate expressDataAngleDelta
	uint32_t packetCount = 0; // a counter for how many packets have been received (used to calculate expressDataAngleDelta)
	uint32_t rotationCount = 0; // a counter to keep track of how many rotations the lidar has made (does NOT get reset when requesting scan start)
	uint32_t rotationSpeedTimer = 0; // a quick'n'dirty timer to calculate rotation speed (using expressDataAngleDelta)
	uint32_t rotationSpeedDt = 0; // (see 1 comment above) this holds the time between packets

	size_t _incompleteBytesRemaining = 0; // if you read only part of a packet (and there's still more to come), keep track of how many bytes are left
	lidarStandardData _incompleteStandardPackets[10]; // this is an array becuase i assume the overhead of calling lidarSerial.readBytes() is substantial
	lidarExpressDataLegacy _incompleteExpressLegacyPacket; // uses 84 bytes to store 32 measurements
	lidarExpressDataExtended _incompleteExpressExtendPacket; // uses 132 bytes to store 96 measurements
	uint32_t _incompleteExpressExtendCabinBuff; // the express extend data structure requires the next datapoint to calculate the current one. This holds the last 4byte 'cabin' value
	lidarExpressDataDense _incompleteExpressDensePacket; // uses 84 bytes to store 40 measurements

	RPlidar(HardwareSerial& lidarSerial ) : lidarSerial(lidarSerial)  {}
	// M�thode pour d�finir le callback
 // M�thode pour d�finir le callback
	void setPostParseCallback(const std::function<void(uint16_t, uint16_t, uint8_t, int8_t)>& callback) {
		postParseCallback = callback;
	}

	void init(uint32_t baudrate = 460800, uint8_t RXpin = 16, uint8_t TXpin = 17) {
		lidarSerial.setRxBufferSize(2048); // increase serial buffer size (becuase the lidar will send a lot of data and sometimes the CPU is busy) (must be done before .begin())
		lidarSerial.begin(baudrate, SERIAL_8N1, RXpin, TXpin);
		lidarSerial.setTimeout(slowSerialTimout);
		delay(25); //wait for the lidar to boot up
		while (lidarSerial.available()) { lidarSerial.read(); } // clear the serial buffer (the lidar prints a little ASCII on boot/reset, i would like to ignore that (running any command should also serve this, but still))
		// test connection by attempting to retrieve scan modes and store them somewhere?
	}
	//private: // oh whatever, i trust the users of the library enough
	void _sendCommand(lidarCMD& commandToSend) {
		lidarSerial.write(commandToSend.send(), commandToSend.sendSize());
		lidarSerial.flush();
	}

	size_t _recvRespDescr(lidarRespDescr& resp, uint32_t startFlagTimeout = 5000) { // read serial data untill both RESP_DESCR_START_FLAGS are received. startFlagTimeout is in micros, returns the number of bytes read
		lidarSerial.setTimeout(slowSerialTimout);
		uint32_t startTime = micros();
		uint8_t syncProgress = 0;
		while (((micros() - startTime) < startFlagTimeout) && (syncProgress < sizeof(RESP_DESCR_START_FLAGS))) {
			if (lidarSerial.available()) {
				uint8_t potentialSyncByte = lidarSerial.read();
				if (potentialSyncByte == RESP_DESCR_START_FLAGS[syncProgress]) {
					syncProgress++;
				}
			}
		}
		if (syncProgress < sizeof(RESP_DESCR_START_FLAGS)) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: _recvRespDescr() "); lidarDebugSerial.print(syncProgress); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(RESP_DESCR_START_FLAGS));
#endif
			return(0); //sync bytes were not received (within startFlagTimeout)
		} // (else) //sync bytes were all received, time to read the message
		return(lidarSerial.readBytes(&resp, sizeof(resp)));
	}

	//size_t wiatForExpressPacketSync() { // TBD(?) a function that looks for the two sync half-bytes at the start of every express packet (and then maybe reads the packet to check the CRC?, although that would also just happen if you let handleData() run)

	lidarRespDescr _sendRequestNoPayload(uint8_t CMDbyte) {
		lidarCMD commandToSend(CMDbyte, 0);
		_sendCommand(commandToSend);
		lidarRespDescr resp;
		size_t bytesRead = _recvRespDescr(resp);
		if (bytesRead < sizeof(resp)) { // if something went wrong with the reading
			//memset(&resp, 0, sizeof(resp)); // set all zeros
			resp.dataType() = RESP_TYPE_INVALID;
			//lidarDebugSerial.print("rplidar: _sendRequestNoPayload() bytesRead < sizeof(resp)"); lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(resp));
		}
		return(resp);
	}

	template<typename returnType, const uint8_t CMDbyte, const uint8_t correct_resp_type_byte> // CMDbyte and correct_resp_type_byte dont need to be templated, theyt could also just be input variables
	returnType _getSomethingSimple() {
		lidarRespDescr resp = _sendRequestNoPayload(CMDbyte);
		returnType returnVal;
		if ((resp.dataType() == correct_resp_type_byte) && (resp.responseLength() == sizeof(returnVal))) {
			lidarSerial.setTimeout(slowSerialTimout);
			size_t bytesRead = lidarSerial.readBytes(&returnVal, sizeof(returnVal));
			if (bytesRead < sizeof(returnVal)) {
				//memset(&returnVal, 0xFF, sizeof(returnVal)); // do something to indicate faillure
#ifdef lidarDebugSerial
				/*lidarDebugSerial.print(funcName);*/ lidarDebugSerial.print("rplidar: _getSomethingSimple() bytesRead < sizeof(returnVal) "); lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(returnVal));
#endif
			}
		}
		else {
			//memset(&returnVal, 0xFF, sizeof(returnVal)); // do something to indicate faillure
#ifdef lidarDebugSerial
			/*lidarDebugSerial.print(funcName);*/ lidarDebugSerial.print("rplidar: _getSomethingSimple() resp wrong! ");
			lidarDebugSerial.print(resp.dataType(), HEX); lidarDebugSerial.print(" != "); lidarDebugSerial.print(correct_resp_type_byte, HEX); lidarDebugSerial.print(" || ");
			lidarDebugSerial.print(resp.responseLength()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(sizeof(returnVal));
#endif
		}
		return(returnVal);
	}


	uint32_t _recvLidarConfRespDesc() {
		lidarRespDescr resp;
		size_t bytesRead = _recvRespDescr(resp);
		if ((bytesRead < sizeof(resp)) || (resp.dataType() != RESP_TYPE_GET_LIDAR_CONF)) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: _recvLidarConfRespDesc() response issue: ");
			lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.print(sizeof(resp)); lidarDebugSerial.print(" || ");
			lidarDebugSerial.print(resp.dataType(), HEX); lidarDebugSerial.print(" != "); lidarDebugSerial.println(RESP_TYPE_GET_LIDAR_CONF, HEX);
#endif
			return(0);
		} // else
		return(resp.responseLength());
	}

	lidarGetLidarConfResponse _recvLidarConfPayload(uint8_t payloadSize) { // payloadSize is INCLUDING 4 bytes for 'type' variable
		lidarGetLidarConfResponse lidarConfPayload(payloadSize - sizeof(uint32_t)); // (all lidarGetLidarConfResponse's include the first 4 bytes, so the constructor just takes how many bytes on top of that are needed)
		lidarSerial.setTimeout(slowSerialTimout);
		size_t bytesRead = lidarSerial.readBytes(&lidarConfPayload, payloadSize);
		if (bytesRead < payloadSize) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: _recvLidarConfPayload() lidarConfPayload read issue: ");
			lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(payloadSize);
#endif
			lidarConfPayload.typeVal() = 0x00; //indicate that receiving failed
		} // else
		return(lidarConfPayload);
	}

	template<typename returnType, const uint8_t CONF_CMDbyte>
	returnType _getLidarConfAttributeBoth(lidarCMD& lidarConfCMD) {
		_sendCommand(lidarConfCMD);
		uint32_t currentRespLength = _recvLidarConfRespDesc();
		if (currentRespLength != (sizeof(uint32_t) + sizeof(returnType))) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: lidarConfCMD currentRespLength bad:"); lidarDebugSerial.println(currentRespLength);
#endif
			return(0);
		} // else
		lidarGetLidarConfResponse lidarConfAttribute = _recvLidarConfPayload(sizeof(uint32_t) + sizeof(returnType));
		if (lidarConfAttribute.typeVal() != CONF_CMDbyte) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: _getLidarConfAttributeBoth() lidarConfAttribute read issue: ");
			lidarDebugSerial.print(lidarConfAttribute.typeVal()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(CONF_CMDbyte);
#endif
			return(0);
		} // else
		return((returnType)lidarConfAttribute);
	}

	template<typename returnType, const uint8_t CONF_CMDbyte>  // CONF_CMDbyte could also just be an input variable and returnType is (currently) always uint16_t
	returnType _getLidarConfAttributeNoPayload() {
		lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t) + 0);
		lidarConfCMD.payloadAsUint32_t(0) = CONF_CMDbyte; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
		return(_getLidarConfAttributeBoth<returnType, CONF_CMDbyte>(lidarConfCMD));
	}

	template<typename returnType, typename requestPayloadType, const uint8_t CONF_CMDbyte>  // CONF_CMDbyte could be an input variable and payloadType is (currently) always uint16_t
	returnType _getLidarConfAttribute(requestPayloadType requestPayload) {
		lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t) + sizeof(requestPayload));
		lidarConfCMD.payloadAsUint32_t(0) = CONF_CMDbyte; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
		requestPayloadType& payloadReference = (requestPayloadType&)lidarConfCMD[sizeof(uint32_t)];
		payloadReference = requestPayload; // should work
		return(_getLidarConfAttributeBoth<returnType, CONF_CMDbyte>(lidarConfCMD));
	}

	uint16_t _varbitscale_decode(uint16_t major, uint8_t& scaleLevel) { // taken from SDK (and modified slightly)
		static const int32_t VBS_SCALED_BASE[] = {
		  3328, // SL_LIDAR_VARBITSCALE_X16_DEST_VAL // thijs: 3328 = 1101 0000 0000 = (13)<<8  ?
		  1792, // SL_LIDAR_VARBITSCALE_X8_DEST_VAL // thijs: 1792 = 0111 0000 0000 = (7)<<8  ?
		  1280, // SL_LIDAR_VARBITSCALE_X4_DEST_VAL // thijs: 1280 = 0101 0000 0000 = (5)<<8  ?
		  512, // SL_LIDAR_VARBITSCALE_X2_DEST_VAL // thijs: 512 = 0010 0000 0000 = (2)<<8  ?
		  0 };
		static const uint16_t VBS_TARGET_BASE[] = {
		  (1 << 14), // SL_LIDAR_VARBITSCALE_X16_SRC_BIT
		  (1 << 12), // SL_LIDAR_VARBITSCALE_X8_SRC_BIT
		  (1 << 11), // SL_LIDAR_VARBITSCALE_X4_SRC_BIT
		  (1 << 9), // SL_LIDAR_VARBITSCALE_X2_SRC_BIT
		  0 };
		for (size_t i = 0; i < 5; ++i) {
			int32_t remain = ((int32_t)major - VBS_SCALED_BASE[i]);
			if (remain >= 0) {
				scaleLevel = 5 - 1 - i;
				return(VBS_TARGET_BASE[i] + (remain << scaleLevel));
			}
		}
		return(0);
	}

public:
	inline lidarGetInfoResponse getInfo() { return(_getSomethingSimple<lidarGetInfoResponse, CMD_GET_INFO, RESP_TYPE_GET_INFO>()); }
	inline lidarGetHealthResponse getHealth() { return(_getSomethingSimple<lidarGetHealthResponse, CMD_GET_HEALTH, RESP_TYPE_GET_HEALTH>()); }
	inline lidarGetSamplerateResponse getSamplerate() { return(_getSomethingSimple<lidarGetSamplerateResponse, CMD_GET_SAMPLERATE, RESP_TYPE_GET_SAMPLERATE>()); }

	void printLidarInfo() {
		lidarGetInfoResponse info = getInfo();
		lidarConsole.print("model: "); lidarConsole.println(info.model());
		lidarConsole.print("firmware: "); lidarConsole.print(info.firmwareMajor()); lidarConsole.print('.'); lidarConsole.println(info.firmwareMinor());
		lidarConsole.print("hardware: "); lidarConsole.println(info.hardware());
		lidarConsole.print("serialNumber (HEX): "); for (uint8_t i = 0; i < 16; i++) { lidarConsole.print(info.serialNumber(i), HEX); lidarConsole.print(' '); } lidarConsole.println();
		lidarConsole.print("serialNumber (32bit): "); for (uint8_t i = 0; i < 4; i++) { lidarConsole.print((uint32_t&)info.serialNumber(i * 4)); lidarConsole.print(' '); } lidarConsole.println();
		lidarConsole.print("serialNumber (64bit): "); for (uint8_t i = 0; i < 2; i++) { lidarConsole.print((uint64_t&)info.serialNumber(i * 8)); lidarConsole.print(' '); } lidarConsole.println();
	}
	void printLidarHealth() {
		lidarGetHealthResponse health = getHealth();
		lidarConsole.print("status: "); lidarConsole.println(health.status());
		lidarConsole.print("errorCode (HEX): "); lidarConsole.println(health.errorCode(), HEX);
	}
	void printLidarSamplerate() {
		lidarGetSamplerateResponse samplerate = getSamplerate();
		lidarConsole.print("standard: "); lidarConsole.println(samplerate.standard());
		lidarConsole.print("express: "); lidarConsole.println(samplerate.express());
	}

	inline uint16_t getLidarConfScanModeCount() { return(_getLidarConfAttributeNoPayload<uint16_t, GET_CONF_SCAN_MODE_COUNT>()); }
	inline uint16_t getLidarConfScanModeTypical() { return(_getLidarConfAttributeNoPayload<uint16_t, GET_CONF_SCAN_MODE_TYPICAL>()); }
	inline uint32_t getLidarConfScanModeSampletime(uint16_t index) { return(_getLidarConfAttribute<uint32_t, uint16_t, GET_CONF_SCAN_MODE_SAMPLETIME>(index)); }
	inline uint32_t getLidarConfScanModeMaxDist(uint16_t index) { return(_getLidarConfAttribute<uint32_t, uint16_t, GET_CONF_SCAN_MODE_MAX_DIST>(index)); }
	inline uint8_t getLidarConfScanModeAnsType(uint16_t index) { return(_getLidarConfAttribute<uint8_t, uint16_t, GET_CONF_SCAN_MODE_ANS_TYPE>(index)); }

	String getLidarConfScanModeName(uint16_t index) { // this one requires an extra special function (mostly becuase the length of the string is not stored in the lidarGetLidarConfResponse struct)
		String returnString = "";
		lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t) + sizeof(index));
		lidarConfCMD.payloadAsUint32_t(0) = GET_CONF_SCAN_MODE_NAME; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
		lidarConfCMD.payloadAsUint16_t(sizeof(uint32_t)) = index;
		_sendCommand(lidarConfCMD);
		uint32_t currentRespLength = _recvLidarConfRespDesc();
		if ((currentRespLength <= sizeof(uint32_t)) || (currentRespLength > 260)) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: lidarConfCMD currentRespLength bad:"); lidarDebugSerial.println(currentRespLength);
#endif
			return(returnString);
		} // else
		lidarGetLidarConfResponse lidarConfAttribute = _recvLidarConfPayload(currentRespLength);
		if (lidarConfAttribute.typeVal() != GET_CONF_SCAN_MODE_NAME) {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: _getLidarConfAttributeBoth() lidarConfAttribute read issue: ");
			lidarDebugSerial.print(lidarConfAttribute.typeVal()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(GET_CONF_SCAN_MODE_NAME);
#endif
			return(returnString);
		} // else
		for (uint32_t i = 0; i < (currentRespLength - sizeof(uint32_t) - 1); i++) { returnString += lidarConfAttribute.charArray(i); } // -1 is to skip the last char (should be 0)
		
		return(returnString);
	}

	void printLidarConfig() {
		uint16_t scanModeCount = getLidarConfScanModeCount();
		//if((scanModeCount == 0) || (scanModeCount > 10)) {lidarConsole.print("");lidarConsole.println();return;}
		lidarConsole.print("scan mode count: "); lidarConsole.println(scanModeCount);
		for (uint16_t i = 0; i < scanModeCount; i++) {
			lidarConsole.print("scan mode: "); lidarConsole.println(i);
			String modeName = getLidarConfScanModeName(i); lidarConsole.print('\t'); lidarConsole.print("name: "); lidarConsole.println(modeName); // inefficient as heck, but it should work
			uint32_t sampletime = getLidarConfScanModeSampletime(i);  lidarConsole.print('\t'); lidarConsole.print("sampletime: "); lidarConsole.print(sampletime); lidarConsole.print(" = "); lidarConsole.print((float)sampletime / 256.0); lidarConsole.println(" microseconds");
			uint32_t maxDist = getLidarConfScanModeMaxDist(i);  lidarConsole.print('\t'); lidarConsole.print("maxDist: "); lidarConsole.print(maxDist); lidarConsole.print(" = "); lidarConsole.print((float)maxDist / 256.0); lidarConsole.println(" meters");
			uint32_t ansType = getLidarConfScanModeAnsType(i);  lidarConsole.print('\t'); lidarConsole.print("ansType (HEX): "); lidarConsole.println(ansType, HEX); // todo: print string for CONF_SCAN_MODE_ANS_TYPE_STANDARD or one of those
		}
		uint16_t scanModeTypical = getLidarConfScanModeTypical();
		lidarConsole.print("scan mode typical: "); lidarConsole.println(scanModeTypical);
	}

	bool startStandardScan(bool force = false) {
		//if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");
		while (lidarSerial.available())
		{
			lidarSerial.read();
		}
		_incompleteBytesRemaining = 0; // reset (and discard) partial packet data
		packetCount = 0; // reset packet counter
		lidarRespDescr resp = _sendRequestNoPayload(force ? CMD_FORCE_SCAN : CMD_SCAN);
		scanResponseFormat = resp.dataType(); // save what type of data the lidar is going to send out
		bool success = (resp.dataType() == RESP_TYPE_SCAN) && (resp.sendMode() == RESP_DESCR_SENDMODE_MULTI_RESPONSE);
		//if((motorHandlerPtr) && success) { //start motor }
		return(success);
	}
	bool startExpressScan(uint8_t extendMode = EXPRESS_SCAN_WORKING_MODE_LEGACY) {
		//if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");

		_incompleteBytesRemaining = 0; // reset (and discard) partial packet data
		packetCount = 0; // reset packet counter
		lidarCMD commandToSend(CMD_EXPRESS_SCAN, 5);
		commandToSend[0] = extendMode; // first byte of CMD payload is 'working mode' (badly described in documentation how these values are derived
		_sendCommand(commandToSend);
		lidarRespDescr resp;
		size_t bytesRead = _recvRespDescr(resp);
		if ((bytesRead < sizeof(resp)) || (resp.sendMode() != RESP_DESCR_SENDMODE_MULTI_RESPONSE) ||     // if something went wrong with the reading
			((resp.dataType() != ((extendMode != EXPRESS_SCAN_WORKING_MODE_LEGACY) ? RESP_TYPE_SCAN_EXPRESS_EXTEND : RESP_TYPE_SCAN_EXPRESS_LEGACY)) && (resp.dataType() != RESP_TYPE_SCAN_EXPRESS_DENSE))) { // or if the datatype is not what you expected
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: startExpressScan error: ");
			lidarDebugSerial.print(bytesRead); lidarDebugSerial.print('<'); lidarDebugSerial.print(sizeof(resp)); lidarDebugSerial.print(" || ");
			lidarDebugSerial.print(resp.sendMode()); lidarDebugSerial.print("!="); lidarDebugSerial.print(RESP_DESCR_SENDMODE_MULTI_RESPONSE); lidarDebugSerial.print(" || ");
			lidarDebugSerial.print(resp.dataType(), HEX); lidarDebugSerial.print("!=");
			if (extendMode != EXPRESS_SCAN_WORKING_MODE_LEGACY) { lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_EXTEND, HEX); }
			else { lidarDebugSerial.print('('); lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_LEGACY, HEX); lidarDebugSerial.print("||"); lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_DENSE, HEX); lidarDebugSerial.print(')'); }
			lidarDebugSerial.println();
#endif
			scanResponseFormat = 0; // save the fact that starting the scan failed
			return(false);
		} // else
		scanResponseFormat = resp.dataType(); // save what type of data the lidar is going to send out
		return(true);
	}

	int8_t handleData(bool includeInvalidMeasurements = true, bool waitForChecksum = false, bool doExtendAngleMath = false) { // returns how many measurements were sent to the postParseCallback()
		if (scanResponseFormat == 0) {
#ifdef lidarDebugSerial
			lidarDebugSerial.println("rplidar: handleData() failed, scanResponseFormat == 0!");
#endif
			return(-1);
		}
		//if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");
		uint8_t measurementsHandled = 0;
		lidarSerial.setTimeout(fastSerialTimout);
		/////////////////////////////////////////////////////////////////////////////////////////////////////////standard///////////////////////////////////////////////////////////////////////////////////
		if (scanResponseFormat == RESP_TYPE_SCAN) { // standard scan data
			if (!_incompleteBytesRemaining) { _incompleteBytesRemaining = sizeof(_incompleteStandardPackets); }
			uint32_t startReadOffset = sizeof(_incompleteStandardPackets) - _incompleteBytesRemaining;
			size_t bytesRead = lidarSerial.readBytes((&_incompleteStandardPackets[0]) + startReadOffset, _incompleteBytesRemaining);
			_incompleteBytesRemaining -= bytesRead;
			uint8_t whereToStopParse = ((sizeof(_incompleteStandardPackets) - _incompleteBytesRemaining) / sizeof(lidarStandardData)); // integer division should ensure automatic low-bound-rounding
			uint8_t whereToStartParse = startReadOffset / sizeof(lidarStandardData); // CHECK
			for (uint8_t i = whereToStartParse; i < whereToStopParse; i++) {
				if (_incompleteStandardPackets[i].checkBit() && ((_incompleteStandardPackets[i].rotStartFlag() != STANDARD_DATA_ROT_START_BITS) && (_incompleteStandardPackets[i].rotStartFlag() != 0))) {
					packetCount++;
					uint16_t dist = _incompleteStandardPackets[i].dist() >> 2; // standard distance data is 16bits, but in 'q2' format. right shifting discards that 0.25mm resolution (but the lidar has a +- 1mm accuracy anyway, 1mm resolution is perfectly adequate
					if (_incompleteStandardPackets[i].rotStartFlag() == 1) { rotationCount++; }
					if ((postParseCallback) && (includeInvalidMeasurements ? true : (dist > 0))) { postParseCallback(dist, _incompleteStandardPackets[i].angle(), (_incompleteStandardPackets[i].rotStartFlag() == 1), _incompleteStandardPackets[i].quality()); }
				}
				else {
#ifdef lidarDebugSerial
					lidarDebugSerial.print("rplidar: handleData() standard scan check error: "); lidarDebugSerial.print(_incompleteStandardPackets[i].checkBit()); lidarDebugSerial.print(" or "); lidarDebugSerial.println(_incompleteStandardPackets[i].rotStartFlag()); return(-1);
#endif
				}
			}
			
		}
		

		else {
#ifdef lidarDebugSerial
			lidarDebugSerial.print("rplidar: handleData() failed, scanResponseFormat unknown: "); lidarDebugSerial.println(scanResponseFormat, HEX);
#endif
			return(-1); /*if(motorHandlerPtr) { //stop motor }*/
		}
		return(measurementsHandled);
	}

	void stopScan() {
		lidarCMD commandToSend(CMD_STOP, 0);
		_sendCommand(commandToSend); // send command (the STOP command triggers no response)
		delay(100);
		while (lidarSerial.available()) { lidarSerial.read(); } // clear the serial buffer (the lidar prints a little ASCII on boot/reset, i would like to ignore that (running any command should also serve this, but still))
	}
	void resetLidar() {
		lidarCMD commandToSend(CMD_RESET, 0);
		_sendCommand(commandToSend); // send command (the STOP command triggers no response)
		while (lidarSerial.available()) { lidarSerial.read(); } // clear the serial buffer (the lidar prints a little ASCII on boot/reset, i would like to ignore that (running any command should also serve this, but still))
		delay(1000); // according to page 15 of the protocol documentation, the lidar needs at least 2ms before it accepts another command
	}
	bool connectionCheck() {
		lidarRespDescr resp = _sendRequestNoPayload(CMD_GET_HEALTH);
		if ((resp.dataType() == RESP_TYPE_GET_HEALTH) && (resp.responseLength() == sizeof(lidarGetHealthResponse))) {
			lidarSerial.setTimeout(slowSerialTimout);
			lidarGetHealthResponse returnVal;
			size_t bytesRead = lidarSerial.readBytes(&returnVal, sizeof(returnVal));
			return(bytesRead == sizeof(returnVal));
		}
		else { return(false); }
	}

	uint32_t rawAnglePerMillisecond() { // i strongly recommend implementing your own RPM measurement system if you want accurate data, this is really more of a rough indication
		if (rotationSpeedDt) { // avoid divide by 0
			uint32_t angleDiff = expressDataAngleDelta * 1000;
			return(angleDiff / rotationSpeedDt);
		}
		else { return(0); }
	}
	float RPM() { // i strongly recommend implementing your own RPM measurement system if you want accurate data, this is really more of a rough indication
		if (rotationSpeedDt) { // avoid divide by 0
			float RPM = expressDataAngleDelta; // angle_q6
			RPM /= rotationSpeedDt; // angle_q6 per micros
			RPM *= (1000000 * 60) / (360 << 6); // angle_q6/micros to RPM
			return(RPM);
		}
		else { return(0); }
	}
};

