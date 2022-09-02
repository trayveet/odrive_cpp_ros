#ifndef ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_
#define ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_

#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <endian.h>

typedef std::vector<uint8_t> commBuffer;

#define ODRIVE_SDK_USB_VENDORID     4617 //decimal for 0x1209
#define ODRIVE_SDK_USB_PRODUCTID_0     3379 // mac
#define ODRIVE_SDK_USB_PRODUCTID_1     3378 // linux?

#define ODRIVE_SDK_PROTOCOL_VERION 1

#define ODRIVE_SDK_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_SDK_TIMEOUT 1000
#define ODRIVE_SDK_MAX_RESULT_LENGTH 100
#define ODRIVE_SDK_LIBUSB_ISERIAL_LENGTH 256

#define ODRIVE_SDK_SCAN_SUCCESS 0
#define ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND 1
#define ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID 2
#define ODRIVE_SDK_UNEXPECTED_RESPONSE 3
#define ODRIVE_SDK_NOT_INITIALIZED 4
#define ODRIVE_SDK_COMM_SUCCESS 0

#define ODRIVE_SDK_SERIAL_NUMBER_CMD 4

#define ODRIVE_SDK_WRITING_ENDPOINT 3 // found with running expore_odrive -v
#define ODRIVE_SDK_READING_ENDPOINT 131 // found with running expore_odrive -v

#define ODRIVE_SDK_MOTOR_NO_ERROR_STATUS 0 // what odrive would return when no motor errors

namespace odrive
{

    class ODriveDriver {

    public:
        ODriveDriver(
                const std::string* odrive_serial_numbers,
                const uint8_t num_odrives,
                const std::string* motor_to_odrive_serial_number_map,
                const uint8_t* motor_index_map,
                const uint8_t num_motors); 
        ~ODriveDriver();

        int init(); // start communication

        int getPosCPR(int motor_index, float &pos);

        int setMotorSpeed(int motor_index, float motor_speed);
        int setMotorSpeeds(float* motor_speeds); // assumed to match num_motors
        
        int setError(int motor_index, uint16_t error_val);
        int setErrors(); // assumed to match num_motors
        
        int setState(int motor_index, uint8_t state_val);
        int setStates(); // assumed to match num_motors
        int setIdleStates(); // idle all motors

        int getMotorSpeed(int motor_index, float &motor_speed);
        int getBusVoltage(int motor_index, float &voltage);

        int getMotorPosition(int motor_index, float &motor_position);

        int readCurrentMotorPosition(int motor_index, int &motor_position);
        int readCurrentMotorPositions(int* axes_positions); // assumed to match num_motors
        
        int checkErrors(uint8_t* error_codes_array); // assumed to match num_motors
        
        int sendWatchdog();

	int setVGain(int motor_index, float gain);
	int setVIGain(int motor_index, float gain);
	int setCCBandwidth(int motor_index, float bandwidth);

    private:
        // read settings
        uint8_t num_odrives_;
        uint8_t num_motors_;

        // saved for use between creation and init
        std::string* odrive_serial_numbers_;
        std::string* motor_to_odrive_serial_number_map_;
        uint8_t* motor_index_map_;

        // for usb
        libusb_device_handle** odrive_handles_;
        libusb_context* libusb_context_;
        uint8_t* motor_to_odrive_handle_index_;

        int initUSBHandlesBySNs();

        short outbound_seq_no_; // unique ids for packets send to odrive
        int getFloat(int motor_index, float &param, int endpoint_id);
        int getInt(int motor_index, int &param, int endpoint_id);

        int odriveEndpointRequest(libusb_device_handle* handle, int endpoint_id, commBuffer& received_payload, int& received_length, const commBuffer payload, const int ack, const int length);
        
        int odriveEndpointGetShort(libusb_device_handle* handle, int endpoint_id, short& value);
        int odriveEndpointGetInt(libusb_device_handle* handle, int endpoint_id, int& value);
        int odriveEndpointGetUInt8(libusb_device_handle* handle, int endpoint_id, uint8_t& value);
        int odriveEndpointGetUInt64(libusb_device_handle* handle, int endpoint_id, uint64_t& value);
        int odriveEndpointGetFloat(libusb_device_handle* handle, int endpoint_id, float& value);
        int odriveEndpointVoid(libusb_device_handle* handle, int endpoint_id);
        
        int odriveEndpointSetInt(libusb_device_handle* handle, int endpoint_id, const int& value);
        int odriveEndpointSetInt8(libusb_device_handle* handle, int endpoint_id, const uint8_t& value);
        int odriveEndpointSetInt16(libusb_device_handle* handle, int endpoint_id, const uint16_t& value);
        int odriveEndpointSetFloat(libusb_device_handle* handle, int endpoint_id, const float& value);
        
        void serializeCommBufferInt(commBuffer& buf, const int& value);
        void serializeCommBufferFloat(commBuffer& buf, const float& value);
       
        void appendShortToCommBuffer(commBuffer& buf, const short value);
        
        void readShortFromCommBuffer(commBuffer& buf, short& value);

        void deserializeCommBufferInt(commBuffer& buf, int& value);
        void deserializeCommBufferUInt64(commBuffer& buf, uint64_t& value);
        void deserializeCommBufferUInt8(commBuffer& buf, uint8_t& value);
        void deserializeCommBufferFloat(commBuffer& buf, float& value);

        commBuffer createODrivePacket(short seq_no, int endpoint, short response_size, const commBuffer& payload_ref);
        commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet);
    };
}

#endif /* ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_ */
