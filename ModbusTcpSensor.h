#include "esphome.h"
#include <WiFiClient.h>


class ModbusClient
{
  static const byte FNC_READ_REGS = 0x03;
  static const byte FNC_WRITE_SINGLE = 0x06;
  static const byte FNC_ERR_FLAG = 0x80;

  static const byte CODE_IX = 7;
  static const byte ERR_CODE_IX = 8;
  static const byte LENGTH_IX = 8;
  static const byte DATA_IX = 9;


  private:
    bool mbWaitingForResponse = false;
    byte mbExpectedRegisterCount = 0;
    int miExpectedResponseLength = 0;
    int miReceivedResponseLength = 0;


  public:
    const int MODBUS_SUCCESS = 0;
    const int MODBUS_IDLE = -1;
    const int MODBUS_CONNECT_ERROR = -10;
    const int MODBUS_WAITING_FOR_RESPONSE = -11;
    const int MODBUS_NO_RESPONSE = -12;
    const int MODBUS_INVALID_FNC = -13;


    ModbusClient()
    {
    }


    /*
    * return
    *   - 0 is success
    *   - negative is communication error
    *   - positive is client protocol exception code
    */

    void beginRequest(WiFiClient& oWifiClient, byte bUnitID, uint16_t uRegistersStartAddress, byte bRegisterCount) 
    {
      oWifiClient.clear();

      byte abRequest[] = {0, 1, 0, 0, 0, 6, bUnitID, FNC_READ_REGS, (byte)((uRegistersStartAddress >> 8) & 0xFF), (byte)(uRegistersStartAddress & 0xFF), 0, bRegisterCount};
      oWifiClient.write(abRequest, sizeof(abRequest));

      mbExpectedRegisterCount = bRegisterCount;
      miExpectedResponseLength = DATA_IX + bRegisterCount * 2;
      miReceivedResponseLength = 0;

      mbWaitingForResponse = true;
    }


    int endRequest(WiFiClient& oWifiClient, short *aiRegisterData)
    {
      if(mbWaitingForResponse)
      {
        if(oWifiClient.available() >= miExpectedResponseLength)
        {
          mbWaitingForResponse = false;

          byte abResponseBytes[miExpectedResponseLength];
          
          int iBytesRead = oWifiClient.readBytes(abResponseBytes, miExpectedResponseLength);
          if (iBytesRead < miExpectedResponseLength) 
          {
            oWifiClient.stop();

            ESP_LOGD("modbus_tcp", "Invalid response: %d", iBytesRead);
            return MODBUS_NO_RESPONSE;
          }

          switch (abResponseBytes[CODE_IX]) 
          {
            case FNC_READ_REGS:
              break;

            case (FNC_ERR_FLAG | FNC_READ_REGS):
              ESP_LOGD("modbus_tcp", "Error response: %d", abResponseBytes[ERR_CODE_IX]);
              return abResponseBytes[ERR_CODE_IX]; // 0x01, 0x02, 0x03 or 0x11

            default:
              ESP_LOGD("modbus_tcp", "Invalid function code: %d", abResponseBytes[CODE_IX]);
              return MODBUS_INVALID_FNC;
          }

          for (int i = 0, j = 0; i < mbExpectedRegisterCount; i++, j += 2) 
          {
            aiRegisterData[i] = (int)(abResponseBytes[DATA_IX + j] << 8 | abResponseBytes[DATA_IX + j + 1]);
          }

          return MODBUS_SUCCESS;
        }
        
        return MODBUS_WAITING_FOR_RESPONSE;
      }
      
      return MODBUS_IDLE;
    }


    // void pDumpBytes(byte *data, int len)
    // {
    //   if(len > 0)
    //   {
    //     String sDataInHex = "";
    //     byte b;
    //     char c;

    //     for (int i = 0; i < len; i++) 
    //     {
    //       b = data[i];

    //       if(b < 0x10)
    //         sDataInHex += "0";

    //       sDataInHex += String(b, HEX) + " ";
    //     }

    //     ESP_LOGD("modbus_tcp", sDataInHex.c_str());
    //   }
    // }
};




class ModbusTcpSensorChild : public Component, public Sensor 
{
public:
  ModbusTcpSensorChild()
  {
  }

  void forceUpdate(float voltage)
  {
    publish_state(voltage);
  }
};




class ModbusTcpSensorContainer : public PollingComponent, public Sensor 
{
public:
  ModbusTcpSensorContainer(const std::string &host, uint16_t port, uint16_t register_address, uint32_t update_interval) : PollingComponent(update_interval), host_(host), port_(port), register_address_(register_address)
  {
  }

  ModbusTcpSensorChild *voltageSensor;
  ModbusTcpSensorChild *powerSensor;

  void setup() override 
  {
  }

  void update() override 
  {
    if(! moWifiClient.connected())
    {
      if (! moWifiClient.connect(host_.c_str(), port_)) 
      {
        ESP_LOGE("modbus_tcp", "Failed to connect to the modbus server %s:%d", host_.c_str(), port_);

        return;
      }
    }

    moWifiClient.setTimeout(1000);
    
    moModbusClient.beginRequest(moWifiClient, 1, 40195, 16);
  }


  void loop() override
  {
    short aiRegisterData[16];
    int iRet = moModbusClient.endRequest(moWifiClient, aiRegisterData);
    if(iRet == 0)
    {
      short iVoltage = aiRegisterData[0];
      short iVoltageScaleFactor = aiRegisterData[8];
      float rVoltage = iVoltage * pow(10, iVoltageScaleFactor);
      voltageSensor->forceUpdate(rVoltage);

      short iPower = aiRegisterData[11];
      short iPowerScaleFactor = aiRegisterData[15];
      float rPower = iPower * pow(10, iPowerScaleFactor);
      powerSensor->forceUpdate(rPower);
    }
  }

private:
  std::string host_;
  uint16_t port_;
  uint16_t register_address_;
  WiFiClient moWifiClient;
  ModbusClient moModbusClient;
};
