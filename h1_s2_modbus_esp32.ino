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

#include <array>
#include <WiFiManager.h>
#include "BLEDevice.h"

//////// Required for Modbus TCP / IP //////////
#define MB_FC_NONE 0
#define MB_FC_READ_REGISTERS 3  // implemented
#define MB_FC_WRITE_REGISTER 6  // implemented

#define MODBUS_TCP_PORT 502

// Defines for Modbus BLE
#define BLE_HEADER_LENGTH 7

// Timeouts
#define WIFI_DISCONNECTED_UNTIL_RESET_TIMEOUT 30000

void(* resetFunc) (void) = 0;  // declare reset function @ address 0


static BLEUUID saj_service_uuid("0000ffff-0000-1000-8000-00805f9b34fb");
static BLEUUID saj_write_characteristic_uuid("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID saj_notify_characteristic_uuid("0000ff02-0000-1000-8000-00805f9b34fb");

WiFiClient client;
WiFiServer modbus_tcp_server(MODBUS_TCP_PORT);


// forward declaration
void handleBleInverterResponse(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify
);


class ModbusTCP {
 protected:
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
  ModbusTCP() {}

  // Static factory method to create object from byte array
  static ModbusTCP fromByteArray(const std::vector<byte>& request) {
    ModbusTCP modbus;

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
    return (transaction_identifier[0] << 8) + transaction_identifier[1] - 1;
  }

  uint16_t getAddress() const {
    return (address[0] << 8) | address[1];
  }

  uint16_t getNumberOfRegisters() const {
    return (number_registers[0] << 8) + (2 * number_registers[1]) + 3;
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
};

class ModbusTCPResponse : public ModbusTCP {
 private:
  std::vector<byte> data;

 public:
  // Static factory method to create object from byte array
  static ModbusTCPResponse fromModbusTCP(const ModbusTCP& modbus_msg) {
    ModbusTCPResponse response;

    const byte *transaction_identifier = modbus_msg.getTransactionIdentifierBytes();
    const byte *protocol_identifier = modbus_msg.getProtocolIdentifierBytes();
    const byte *address = modbus_msg.getAddressBytes();
    uint16_t number_registers = modbus_msg.getNumberOfRegisters();
    const byte *number_register_bytes = modbus_msg.getNumberRegistersBytes();

    // Copy header fields
    response.transaction_identifier[0] = transaction_identifier[0];
    response.transaction_identifier[1] = transaction_identifier[1];
    response.protocol_identifier[0] = protocol_identifier[0];
    response.protocol_identifier[1] = protocol_identifier[1];
    response.length[0] = (number_registers & 0x0000ff00) >> 8;
    response.length[1] = number_registers & 0x000000ff;
    response.unit_id = modbus_msg.getUnitId();
    response.function_code = modbus_msg.getFunctionCode();
    response.number_registers[0] = number_register_bytes[0];
    response.number_registers[1] = number_register_bytes[1];

    return response;
  }

  void setData(const std::vector<byte>& data) { this->data = data; }

  std::vector<byte> toByteVector() const {
    std::vector<byte> response = {
      transaction_identifier[0],
      transaction_identifier[1],
      protocol_identifier[0],
      protocol_identifier[1],
      length[0],
      length[1],
      unit_id,
      function_code
    };

    response.insert(response.end(), data.begin(), data.end());
    return response;
  }
};

class ModbusBLERequest {
 private:
  static byte ble_transaction_id;

  // Modbus BLUE header fields
  byte ble_transaction_identifier;
  byte transaction_identifier[2];
  byte unit_id;
  byte function_code;
  byte address[2];
  byte number_registers[2];

  // Calculate ModRTU CRC for this modbus request
  uint16_t getModRTU_CRC() const {
    byte modbus_message_final[6] = {
      unit_id,
      function_code,
      address[0],
      address[1],
      number_registers[0],
      number_registers[1]
    };

    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < 6; pos++) {
      crc ^= (uint16_t)modbus_message_final[pos];  // XOR byte into least sig. byte of crc

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

 public:
  static ModbusBLERequest fromModbusTCP(const ModbusTCP& request) {
    ModbusBLERequest modbus;

    modbus.ble_transaction_identifier = ModbusBLERequest::ble_transaction_id;
    modbus.transaction_identifier[0] = request.getTransactionIdentifierBytes()[0];
    modbus.transaction_identifier[1] = request.getTransactionIdentifierBytes()[1];
    modbus.unit_id = request.getUnitId();
    modbus.function_code = request.getFunctionCode();
    modbus.address[0] = request.getAddressBytes()[0];
    modbus.address[1] = request.getAddressBytes()[1];
    modbus.number_registers[0] = request.getNumberRegistersBytes()[0];
    modbus.number_registers[1] = request.getNumberRegistersBytes()[1];

    ModbusBLERequest::ble_transaction_id++;

    return modbus;
  }

  std::array<byte, 13> toByteArray() const {
    uint16_t crc = getModRTU_CRC();
    byte high_byte = crc >> 8;
    byte low_byte = crc & 0xFF;
    
    std::array<byte, 13> request = {
      77,
      0,
      ble_transaction_identifier,
      9,
      50,
      unit_id,
      function_code,
      address[0],
      address[1],
      number_registers[0],
      number_registers[1],
      low_byte,
      high_byte,
    };

    return request;
  }
};

class ModbusBLEResponse {
 private:
  std::vector<byte> data;

 public:
  static ModbusBLEResponse fromByteArray(byte* data, size_t length) {
    ModbusBLEResponse response;
    size_t data_length = data[BLE_HEADER_LENGTH] + 1;
    byte *data_start_ptr = data + BLE_HEADER_LENGTH;
    response.data.assign(data_start_ptr, data_start_ptr + data_length);

    return response;
  }

  std::vector<byte>& getData() { return data; }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}

  void onDisconnect(BLEClient* pclient) {
    Serial.println("Reset 2. Bluetooth client disconnected");
    resetFunc();
  }
};

/**
 * BLE Connection Manager
 * Manages the entire BLE lifecycle: scanning, discovery, connection, and state management
 */
class SajInverterBleManager {
 public:
  enum ConnectionState {
    DISCONNECTED,
    SCANNING,
    DEVICE_FOUND,
    CONNECTING,
    CONNECTED,
    CONNECTION_FAILED
  };

 private:
  ConnectionState current_state;
  BLEClient* ble_client;
  BLEAdvertisedDevice discovered_device;
  BLERemoteCharacteristic* notify_characteristic;
  BLERemoteCharacteristic* write_characteristic;
  bool device_discovered;
  
  // Inner class for device discovery
  class InverterDiscovery : public BLEAdvertisedDeviceCallbacks {
   private:
    SajInverterBleManager* manager;
    
   public:
    InverterDiscovery(SajInverterBleManager* mgr) : manager(mgr) {}
    
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
      if (advertisedDevice.haveServiceUUID() && 
          advertisedDevice.isAdvertisingService(saj_service_uuid)) {
        Serial.println("SAJ Inverter found!");
        BLEDevice::getScan()->stop();
        manager->onDeviceDiscovered(advertisedDevice);
      }
    }
  };
  
  // Inner class for client callbacks
  class ClientCallbacks : public BLEClientCallbacks {
   private:
    SajInverterBleManager* manager;
    
   public:
    ClientCallbacks(SajInverterBleManager* mgr) : manager(mgr) {}
    
    void onConnect(BLEClient* client) override {
      Serial.println("BLE Client connected");
    }
    
    void onDisconnect(BLEClient* client) override {
      Serial.println("BLE Client disconnected - resetting system");
      manager->current_state = DISCONNECTED;
      resetFunc();
    }
  };

public:
  SajInverterBleManager() : 
    current_state(DISCONNECTED),
    ble_client(nullptr),
    notify_characteristic(nullptr),
    write_characteristic(nullptr),
    device_discovered(false) {
  }
  
  void initialize() {
    Serial.println("Initializing BLE Manager...");
    BLEDevice::init("");
    ble_client = BLEDevice::createClient();
    ble_client->setClientCallbacks(new ClientCallbacks(this));
    current_state = DISCONNECTED;
  }
  
  void startScanning() {
    if (current_state != DISCONNECTED && current_state != CONNECTION_FAILED) {
      return; // Already scanning or connected
    }
    
    Serial.println("Starting BLE scan for SAJ inverter...");
    current_state = SCANNING;
    device_discovered = false;
    
    BLEScan* scanner = BLEDevice::getScan();
    scanner->setAdvertisedDeviceCallbacks(new InverterDiscovery(this));
    scanner->setInterval(1349);
    scanner->setWindow(449);
    scanner->setActiveScan(true);
    scanner->start(5000, false);
  }
  
  void onDeviceDiscovered(BLEAdvertisedDevice& device) {
    discovered_device = device;
    device_discovered = true;
    current_state = DEVICE_FOUND;
    Serial.println("Device discovered, will connect on next update");
  }
  
  void update() {
    switch (current_state) {
      case DISCONNECTED:
      case CONNECTION_FAILED:
        startScanning();
        break;
        
      case SCANNING:
        // Wait for discovery callback
        break;
        
      case DEVICE_FOUND:
        attemptConnection();
        break;
        
      case CONNECTING:
        // Connection in progress
        break;
        
      case CONNECTED:
        // Nothing to do, stay connected
        break;
    }
  }
  
  bool isConnected() const {
    return current_state == CONNECTED;
  }
  
  bool isReady() const {
    return isConnected() && notify_characteristic && write_characteristic;
  }
  
  ConnectionState getState() const {
    return current_state;
  }
  
  const char* getStateString() const {
    switch (current_state) {
      case DISCONNECTED: return "DISCONNECTED";
      case SCANNING: return "SCANNING";
      case DEVICE_FOUND: return "DEVICE_FOUND";
      case CONNECTING: return "CONNECTING";
      case CONNECTED: return "CONNECTED";
      case CONNECTION_FAILED: return "CONNECTION_FAILED";
      default: return "UNKNOWN";
    }
  }
  
  bool writeToInverter(const uint8_t* data, size_t length) {
    if (!isReady()) {
      Serial.println("BLE Manager not ready for writing");
      return false;
    }
    
    return write_characteristic->writeValue(const_cast<uint8_t*>(data), length, true);
  }

private:
  void attemptConnection() {
    if (!device_discovered) {
      current_state = CONNECTION_FAILED;
      return;
    }
    
    Serial.println("Attempting BLE connection...");
    current_state = CONNECTING;
    
    try {
      // Connect to device
      if (!ble_client->connect(&discovered_device)) {
        Serial.println("Failed to connect to device");
        current_state = CONNECTION_FAILED;
        return;
      }
      
      // Set MTU
      ble_client->setMTU(512);
      Serial.println("Connected to SAJ inverter");
      
      // Get service
      BLERemoteService* service = ble_client->getService(saj_service_uuid);
      if (!service) {
        Serial.println("Failed to find service");
        ble_client->disconnect();
        current_state = CONNECTION_FAILED;
        return;
      }
      
      // Get characteristics
      notify_characteristic = service->getCharacteristic(saj_notify_characteristic_uuid);
      write_characteristic = service->getCharacteristic(saj_write_characteristic_uuid);
      
      if (!notify_characteristic || !write_characteristic) {
        Serial.println("Failed to find characteristics");
        ble_client->disconnect();
        current_state = CONNECTION_FAILED;
        return;
      }
      
      // Register for notifications
      if (notify_characteristic->canNotify()) {
        notify_characteristic->registerForNotify(handleBleInverterResponse);
      }
      
      current_state = CONNECTED;
      Serial.println("BLE connection fully established");
      
    } catch (...) {
      Serial.println("Exception during BLE connection");
      current_state = CONNECTION_FAILED;
    }
  }
};


// BLE Connection Manager - replaces all individual BLE global variables
static SajInverterBleManager saj_ble_manager;
static ModbusTCPResponse modbus_tcp_response;

// Global vars
static uint64_t wifi_disconnect_timestamp_ms = 0;
static int errlen = 0;
static int loop_iterations = 0;

// Initialization of static members
byte ModbusBLERequest::ble_transaction_id = 0;


void print_array(const byte* data, int length, String title) {
    Serial.println("");
    String sdata = title + ": [";
    for (int j = 0; j < length - 1; j++)
      sdata = sdata + (String) data[j] + ", ";

    sdata = sdata + (String) data[length - 1] + "]";
    Serial.print(sdata);
    Serial.println("");
}

void handleBleInverterResponse(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify
) {

  if (modbus_tcp_response.getFunctionCode() == 6) {  // function code
    Serial.println("Update completed?");
    client.clear();
    return;
  }
  
  // Expected to receive the number of requested registers. In the response there are 7 more bytes than requested
  if ((static_cast<int>(length) - 7) != modbus_tcp_response.getNumberOfRegisters()) {
    Serial.println("ERROR DETECTED");
    errlen++;
    String error_detail = (String)"Requested registers: " + modbus_tcp_response.getNumberOfRegisters() + \
      (String)". Received: " + (String)(length-7);
    Serial.println(error_detail);
    print_array(pData, length, "RESPONSE ERROR <25 BYTES");
    if (errlen > 5) {
      Serial.println("Reset 1. Multiple erroneous messages received (5)");
      resetFunc();
    }

    return;
  } 

  errlen = 0;
  print_array(pData, length, "data received from BLE");

  ModbusBLEResponse ble_response = ModbusBLEResponse::fromByteArray(pData, length);
  modbus_tcp_response.setData(ble_response.getData());
  std::vector<byte> serialized_response = modbus_tcp_response.toByteVector();

  client.write(serialized_response.data(), serialized_response.size());
  client.clear();
}

// Old connectToInverterBle function removed - functionality moved to SajInverterBleManager

void wifiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to WIFI successfully!");
  modbus_tcp_server.begin();
  setupBleConnection();
}

void wifiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(MODBUS_TCP_PORT);
  Serial.println("Modbus TCP/IP Online");
}

void wifiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  wifi_disconnect_timestamp_ms = millis();
}

void setupWifiManager() {
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(wifiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(wifiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(wifiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  
  WiFiManager wm;
  wm.setConnectTimeout(300);

  bool res;
  res = wm.autoConnect("ESP32_SAJ_MODBUS");

  if (!res)
        Serial.println("Failed to connect");
  else
      Serial.println("connected...yeey :)");
}

void setupBleConnection() {
  // Initialize the BLE manager
  saj_ble_manager.initialize();
  // Start scanning for SAJ inverter
  saj_ble_manager.startScanning();
}

void setup() {
  Serial.begin(115200);
  setupWifiManager();
}

void loop() {
  loop_iterations++;
  Serial.print("loop_iteration number: ");
  Serial.print(loop_iterations);
  Serial.println("");

  if ((WiFi.status() != WL_CONNECTED) && (millis() - wifi_disconnect_timestamp_ms >= WIFI_DISCONNECTED_UNTIL_RESET_TIMEOUT)) {
    Serial.println("Reset 3. No WiFi connectivity for more than 30 seconds");
    resetFunc();
  }
  
  // Update BLE connection manager state
  saj_ble_manager.update();
  
  // Check if BLE is ready for Modbus communication
  if (!saj_ble_manager.isConnected()) {
    Serial.print("BLE Status: ");
    Serial.println(saj_ble_manager.getStateString());
    return; // Wait for BLE connection before processing Modbus requests
  }

  client = modbus_tcp_server.accept();
  if (!client)
        return;
  
  // Modbus TCP/IP
  while (client.connected()) {
    if (client.available()) {
      Serial.println("client.available");
      std::vector<byte> modbus_request;

      while (client.available())
        modbus_request.push_back(client.read());

      client.clear();
      Serial.print("TCP request received. Length: ");
      Serial.println(modbus_request.size());

      ModbusTCP request = ModbusTCP::fromByteArray(modbus_request);
      request.printDebugInfo();

      ModbusBLERequest ble_request = ModbusBLERequest::fromModbusTCP(request);
      modbus_tcp_response = ModbusTCPResponse::fromModbusTCP(request);

      Serial.print("Sending read message to BT - FN: ");
      Serial.println(request.getFunctionCode());
      std::array<byte, 13> ble_request_array = ble_request.toByteArray();
      print_array(ble_request_array.data(), sizeof(ble_request_array), "ble_request_array");

      switch (request.getFunctionCode()) {
          case MB_FC_NONE:
            break;

        case MB_FC_READ_REGISTERS:  // 03 Read Holding Registers
        case MB_FC_WRITE_REGISTER:  // 06 Write Holding Register
            // 50:   32 on hexadecimal view on the app
            // 1:    01 The unit identifier
            // 3:    03 The function code
            // 64:   40 address
            // 105:  69 address
            // 0:    00 length data
            // 14:   0E length data
            // 1:    01 crc
            // 210:  D2 crc
          if (!saj_ble_manager.writeToInverter(ble_request_array.data(), 13))
            Serial.println("Failed to write to BLE characteristic");

          break;
      }
    }
  }
}
