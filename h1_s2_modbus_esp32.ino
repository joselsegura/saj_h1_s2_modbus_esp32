/*
 * SAJ H1 S2 Modbus TCP Bridge for ESP32
 *
 * This project enables Modbus TCP access to SAJ H1 S2 solar inverters by creating
 * a wireless bridge between Modbus TCP clients and the inverter's Bluetooth interface.
 *
 * Copyright (c) 2024-2025 SAJ H1 S2 Modbus ESP32 Contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Project Repository: https://github.com/sgsancho/saj_h1_s2_modbus_esp32
 * Community: https://t.me/saj_nooficialoriginal
 *
 * DISCLAIMER: This is an unofficial, community-developed project.
 * It is not officially supported by SAJ and may stop working with firmware updates.
 * Use at your own risk.
 */

#include <WiFiManager.h>
#include "BLEDevice.h"

uint64_t previousMillis = 0;
uint64_t previousMillistry = 0;

static BLEUUID serviceUUID("0000ffff-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUIDw("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("0000ff02-0000-1000-8000-00805f9b34fb");

static boolean doConnect_ble = false;
static boolean connected_ble = false;
static boolean doScan_ble = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicw;
static BLEAdvertisedDevice* myDevice;

BLEClient*  pClient  = BLEDevice::createClient();

bool errorr = false;
bool waitingmessage = false;

int soft = 0;
byte modbus_frame_response[8];
int total_registros;

#define UInt16 uint16_t

int ModbusTCP_port = 502;
WiFiServer MBServer(ModbusTCP_port);

//////// Required for Modbus TCP / IP //////////
#define MB_FC_NONE 0
#define MB_FC_READ_REGISTERS 3  // implemented
#define MB_FC_WRITE_REGISTER 6  // implemented

WiFiClient client;
byte mosbus_request[260];

int errlen = 0;

void(* resetFunc) (void) = 0;  // declare reset function @ address 0

byte c = 0;
int total_calls = 0;
int total_errors = 0;

uint64_t previousMillis_wifi = 0;


class ModbusTCPRequest {
 private:
  // Modbus TCP header fields
  byte transaction_identifier[2];
  byte protocol_identifier[2];
  byte length[2];
  byte unit_id;
  byte function_code;
  byte address[2];
  byte number_registers[2];

 public:
  // Constructor
  ModbusTCPRequest() {}

  // Static factory method to create object from byte array
  static ModbusTCPRequest fromByteArray(const byte* request) {
    ModbusTCPRequest modbus;

    // Parse header fields
    modbus.transaction_identifier[0] = request[0];
    modbus.transaction_identifier[1] = request[1];
    modbus.protocol_identifier[0] = request[2];
    modbus.protocol_identifier[1] = request[3];
    modbus.length[0] = request[4];
    modbus.length[1] = request[5];
    modbus.unit_id = request[6];
    modbus.function_code = request[7];
    modbus.address[0] = request[8];
    modbus.address[1] = request[9];
    modbus.number_registers[0] = request[10];
    modbus.number_registers[1] = request[11];

    return modbus;
  }

  // Getter methods for accessing parsed fields
  byte getFunctionCode() const { return function_code; }
  byte getUnitId() const { return unit_id; }

  uint16_t getTransactionId() const {
    return (transaction_identifier[0] << 8) | transaction_identifier[1];
  }

  uint16_t getAddress() const {
    return (address[0] << 8) | address[1];
  }

  uint16_t getNumberOfRegisters() const {
    return (number_registers[0] << 8) | number_registers[1];
  }

  // Get raw byte arrays (by reference - no copying!)
  const byte* getTransactionIdentifierBytes() const { return transaction_identifier; }
  const byte* getProtocolIdentifierBytes() const { return protocol_identifier; }
  const byte* getLengthBytes() const { return length; }
  const byte* getAddressBytes() const { return address; }
  const byte* getNumberRegistersBytes() const { return number_registers; }

  // Debug method
  void printDebugInfo() const {
    Serial.println("");
    Serial.print("modbus_transaction_identifier: ");
    Serial.print(transaction_identifier[0]);
    Serial.print(" ");
    Serial.println(transaction_identifier[1]);

    Serial.print("modbus_protocol_identifier: ");
    Serial.print(protocol_identifier[0]);
    Serial.print(" ");
    Serial.println(protocol_identifier[1]);

    Serial.print("modbus_length: ");
    Serial.print(length[0]);
    Serial.print(" ");
    Serial.println(length[1]);

    Serial.print("modbus_unit_id: ");
    Serial.println(unit_id);

    Serial.print("modbus_function_code: ");
    Serial.println(function_code);

    Serial.print("modbus_address: ");
    Serial.print(address[0]);
    Serial.print(" ");
    Serial.println(address[1]);

    Serial.print("modbus_number_registers: ");
    Serial.print(number_registers[0]);
    Serial.print(" ");
    Serial.println(number_registers[1]);
  }

  // Calculate ModRTU CRC for this modbus request
  UInt16 getModRTU_CRC() const {
    byte modbus_message_final[6] = {
      unit_id,
      function_code,
      address[0],
      address[1],
      number_registers[0],
      number_registers[1]
    };

    UInt16 crc = 0xFFFF;

    for (int pos = 0; pos < 6; pos++) {
      crc ^= (UInt16)modbus_message_final[pos];  // XOR byte into least sig. byte of crc

      for (int i = 8; i != 0; i--) {    // Loop over each bit
        if ((crc & 0x0001) != 0) {      // If the LSB is set
          crc >>= 1;                    // Shift right and XOR 0xA001
          crc ^= 0xA001;
        } else {                           // Else LSB is not set
          crc >>= 1;                    // Just shift right
        }
      }
    }

    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
  }
};

void print_array(byte data[], int length, String title) {
    Serial.println("");
    String sdata = title + ": [";
    for (int j = 0; j < length; j++)
      sdata = sdata + (String)(int8_t) data[j]+",";

    sdata = sdata + "]";
    Serial.print(sdata);
    Serial.println("");
}

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify
) {
  waitingmessage = false;

  if (modbus_frame_response[7] == 6) {  // function code
    Serial.println("Update completed?");
    client.clear();
  } else {
    // Expected to receive the number of requested registers. In the response there are 7 more bytes than requested
    if ((static_cast<int>(length) - 7) != total_registros) {
      Serial.println("ERROR DETECTED");
      errlen = errlen + 1;
      String error_detail = (String)"Requested registers: " + total_registros + \
        (String)". Received: " + (String)(length-7);
      Serial.println(error_detail);
      print_array(pData, length, "RESPONSE ERROR <25 BYTES");
      if (errlen > 5) {
        Serial.println("Reset 1. Multiple erroneous messages received (5)");
        resetFunc();
      } else {
        previousMillistry = millis();
      }
    } else {
      errlen = 0;
      previousMillistry = millis();
      int longitud = abs(pData[7]);  // 20
      byte slice[longitud+1];
      memcpy(slice, pData + 7, longitud+1);
      slice[0] = longitud;

      byte combined[longitud+9];
      memcpy(combined, modbus_frame_response, 8);
      memcpy(&combined[8], slice, longitud+1);

      combined[1] = modbus_frame_response[1];

      print_array(combined, length+8, "Response final");

      client.write(combined, longitud+9);
      client.clear();
    }
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}

  void onDisconnect(BLEClient* pclient) {
    Serial.println("Reset 2. Bluetooth client disconnected");
    resetFunc();
  }
};

bool connectToServer_ble() {
  BLEDevice::getScan()->stop();
  connected_ble = true;

  Serial.println("Forming a connection to ");
  previousMillistry = millis();
  Serial.println(myDevice->getAddress().toString().c_str());

  Serial.println(" - Created client");
  pClient->setClientCallbacks(new MyClientCallback());

  previousMillistry = millis();
  pClient->connect(myDevice);
  pClient->setMTU(512);  // mtu size equals packet limit-3 250
  Serial.println(" - Connected to server");

  previousMillistry = millis();

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  Serial.println(" - Found our service");
  previousMillistry = millis();

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  previousMillistry = millis();
  pRemoteCharacteristicw = pRemoteService->getCharacteristic(charUUIDw);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  previousMillistry = millis();
  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  soft = 0;
  doConnect_ble = false;
  return true;
}
/* Scan for BLE servers and find the first one that advertises the service we are looking for.*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  /* Called for each advertising BLE server. */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect_ble = true;
      doScan_ble = true;
    }
  }
};

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to WIFI successfully!");
  MBServer.begin();
  setup_ble();
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(String(ModbusTCP_port));
  Serial.println("Modbus TCP/IP Online");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
}

void setup_wifi_manager() {
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFiManager wm;
  wm.setConnectTimeout(300);

  bool res;
  res = wm.autoConnect("ESP32_SAJ_MODBUS");

  if (!res)
    Serial.println("Failed to connect");
  else
    Serial.println("connected...yeey :)");
}

void setup_ble() {
  /////////////////////  BLUFI  ///////////////////////
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5000, false);
}

void setup() {
  Serial.begin(115200);
  setup_wifi_manager();
}

void loop() {
  uint64_t currentMillis_wifi = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis_wifi - previousMillis_wifi >=30000)) {
    Serial.println("Reset 3. No WiFi connectivity for more than 30 seconds");
    resetFunc();
  }

  if (doConnect_ble) {
    if (connectToServer_ble())
      Serial.println("We are now connected to the BLE Server.");
    else
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
  }

  uint64_t currentMillis = millis();

  if (connected_ble) {
      client = MBServer.accept();
      if (!client)
        return;

      byte byteFN = MB_FC_NONE;

      // Modbus TCP/IP
      while (client.connected()) {
      if (client.available()) {
        if (c > 255)
          c = 0;

        Serial.println("client.available");
        int i = 0;
        while (client.available()) {
          mosbus_request[i] = client.read();
          i++;
        }
        client.clear();
        Serial.println("TCP request received");

        previousMillis = currentMillis;
        waitingmessage = true;
        ModbusTCPRequest request = ModbusTCPRequest::fromByteArray(mosbus_request);
        request.printDebugInfo();

        byteFN = mosbus_request[7];

        Serial.println("");
        Serial.print("c: ");
        Serial.print(c);

        Serial.println("");
        Serial.print("total_calls: ");
        Serial.print(total_calls);
        Serial.println("");

        UInt16 crc = request.getModRTU_CRC();

        unsigned char high_byte = crc >> 8;
        unsigned char low_byte = crc & 0xFF;

        int transaction_identifier = ((request.getTransactionIdentifierBytes()[0] << 8) +
          (request.getTransactionIdentifierBytes()[1])) - 1;

        byte transaction_identifier_hex[2];
        transaction_identifier_hex[0] = (transaction_identifier & 0x0000ff00) >> 8;
        transaction_identifier_hex[1] = transaction_identifier & 0x000000ff;

        byte mosbus_request_final[13] = {
        (byte) 77,
          0,
          c,
          (byte) 9,
          (byte) 50,
          request.getUnitId(),  // modbus_unit_id
          request.getFunctionCode(),  // modbus_function_code
          request.getAddressBytes()[0],  // modbus_address
          request.getAddressBytes()[1],  // modbus_address
          request.getNumberRegistersBytes()[0],  // modbus_number_registers
          request.getNumberRegistersBytes()[1],  // modbus_number_registers
          (byte) low_byte,  // crc
          (byte) high_byte  // crc
        };

        total_registros = ((request.getNumberRegistersBytes()[0] << 8) +
          (request.getNumberRegistersBytes()[1]) * 2) + 3;

        byte total_registros_hex[2];
        total_registros_hex[0] = (total_registros & 0x0000ff00) >> 8;
        total_registros_hex[1] = total_registros & 0x000000ff;

        modbus_frame_response[0] = (byte) mosbus_request[0];
        modbus_frame_response[1] = (byte) mosbus_request[1];
        modbus_frame_response[2] = (byte) mosbus_request[2];
        modbus_frame_response[3] = (byte) mosbus_request[3];
        modbus_frame_response[4] = (byte) total_registros_hex[0];
        modbus_frame_response[5] = (byte) total_registros_hex[1];
        modbus_frame_response[6] = (byte) mosbus_request[6];
        modbus_frame_response[7] = (byte) mosbus_request[7];

        Serial.print("Sending read message to BT: ");
        Serial.println(byteFN);
        print_array(mosbus_request_final, sizeof(mosbus_request_final), "mosbus_request_final");

        switch (byteFN) {
          case MB_FC_NONE:
            break;

          case MB_FC_READ_REGISTERS:  // 03 Read Holding Registers

            // 50:   32 on hexadecimal view on the app
            // 1:    01 The unit identifier
            // 3:    03 The function code
            // 64:   40 address
            // 105:  69 address
            // 0:    00 length data
            // 14:   0E length data
            // 1:    01 crc
            // 210:  D2 crc
            pRemoteCharacteristicw->writeValue(mosbus_request_final, 13, true);
            byteFN = MB_FC_NONE;
            break;

          case MB_FC_WRITE_REGISTER:  // 06 Write Holding Register
            pRemoteCharacteristicw->writeValue(mosbus_request_final, 13, true);
            byteFN = MB_FC_NONE;
            break;
          }

          c = c+1;
          total_calls = total_calls +1;
        }
      }
  } else if (doScan_ble) {
    Serial.println("scanning again");

    doConnect_ble = true;
    connected_ble = false;
  }
}
