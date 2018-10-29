/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_driver.h"

static dynamixel::PacketHandler *packetHandler_1 = dynamixel::PacketHandler::getPacketHandler(1.0f);
static dynamixel::PacketHandler *packetHandler_2 = dynamixel::PacketHandler::getPacketHandler(2.0f);

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), sync_write_handler_cnt_(0), sync_read_handler_cnt_(0) {}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      writeRegister(tools_[i].getID()[j], "Torque_Enable", (uint8_t)0);
    }
  }

  for (int i = 0; i < sync_write_handler_cnt_; i++)
  {
    delete[] syncWriteHandler_[i].groupSyncWrite;
  }  

  for (int i = 0; i < sync_read_handler_cnt_; i++)
  {
    delete[] syncReadHandler_[i].groupSyncRead;
  }  

  portHandler_->closePort();
}

void DynamixelDriver::initTools(void)
{
  tools_cnt_ = 0;

  for (uint8_t num = 0; num < tools_cnt_; num++)
    tools_[num].initTool();
}

bool DynamixelDriver::setTool(uint16_t model_number, uint8_t id, const char **log)
{
  bool result = false;

  // See if we have a matching tool? 
  for (uint8_t num = 0; num < tools_cnt_; num++)
  {
    if (tools_[num].getModelNumber() == model_number)
    {
      if (tools_[num].getDynamixelCount() < tools_[num].getDynamixelBuffer())
      {
        // Found one with the right model number and it is not full
        tools_[num].addDXL(model_number, id);
        return true;
      }
      else
      {
        *log = "[DynamixelDriver] Too many Dynamixels are connected (default buffer size is 16, the same series of Dynamixels)";
        return false;
      }
    }
  }
  // We did not find one so lets allocate a new one
  if (tools_cnt_ < MAX_DXL_SERIES_NUM) 
  {
    // only do it if we still have some room...
    result = tools_[tools_cnt_++].addTool(model_number, id, log);
    return result;
  }
  else
  {
    *log = "[DynamixelDriver] Too many series are connected (MAX = 5 different series)";
    return false;
  }

  *log = "[DynamixelDriver] Failed to set the Tool";
  return false;
}

uint8_t DynamixelDriver::getTool(uint8_t id, const char **log)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      if (tools_[i].getID()[j] == id)
      {
        return i;
      }
    }
  }

  *log = "[DynamixelDriver] Failed to get the Tool";
  return 0xff;
}

bool DynamixelDriver::init(const char *device_name, uint32_t baud_rate, const char **log)
{
  bool result = false;

  result = setPortHandler(device_name, log);
  if (result == false) return false;

  result = setBaudrate(baud_rate, log);
  if (result == false) return false;

  return result;
}

bool DynamixelDriver::setPortHandler(const char *device_name, const char **log)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
    *log = "[DynamixelDriver] Succeeded to open the port!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to open the port!";
  return false;
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate, const char **log)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
    *log = "[DynamixelDriver] Succeeded to change the baudrate!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to change the baudrate!";
  return false;
}

bool DynamixelDriver::setPacketHandler(float protocol_version, const char **log)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    *log = "[DynamixelDriver] Succeeded to set the protocol!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to set the protocol!";
  return false;
}

float DynamixelDriver::getProtocolVersion(void)
{
  return packetHandler_->getProtocolVersion();
}

uint32_t DynamixelDriver::getBaudrate(void)
{
  return portHandler_->getBaudRate();
}

const char* DynamixelDriver::getModelName(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);

  if (factor == 0xff) 
    return NULL;
  else
    return tools_[factor].getModelName();

  return NULL;
}

uint16_t DynamixelDriver::getModelNumber(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return 0;

  for (int i = 0; i < tools_[factor].getDynamixelCount(); i++)
  {
    if (tools_[factor].getID()[i] == id)
      return tools_[factor].getModelNumber();
  }

  return 0;
}

const ControlItem* DynamixelDriver::getControlTable(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL;

  return tools_[factor].getControlTable();
}

uint8_t DynamixelDriver::getTheNumberOfControlItem(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return 0;

  return tools_[factor].getTheNumberOfControlItem();
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t range, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint8_t id = 0;
  uint8_t id_cnt = 0;

  uint16_t model_number = 0;

  uint8_t get_range = range;

  if (get_range > 253) get_range = 253;

  initTools();

  for (id = 0; id <= get_range; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_1->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }    
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    result = setPacketHandler(1.0f, log);
    return result;
  }

  for (id = 0; id <= get_range; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_2->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }   
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    result = setPacketHandler(2.0f, log);
    return result;
  }

  *log = "[DynamixelDriver] Failed to scan!";
  return false;
}

bool DynamixelDriver::ping(uint8_t id, uint16_t *get_model_number, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint16_t model_number = 0;

  sdk_error.dxl_comm_result = packetHandler_1->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(1.0f, log);
    return result;
  }

  sdk_error.dxl_comm_result = packetHandler_2->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(2.0f, log);
    return result;
  }  

  *log = "[DynamixelDriver] Failed to ping!";
  return false;
}

bool DynamixelDriver::reboot(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  if (getProtocolVersion() == 1.0)
  {
    *log = "[DynamixelDriver] reboot functions is not available with the Dynamixel Protocol 1.0.";
    return false;
  }
  else
  {
    sdk_error.dxl_comm_result = packetHandler_2->reboot(portHandler_, id, &sdk_error.dxl_error);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      *log = "[DynamixelDriver] Succeeded to reboot!";
      return true;
    }
  }

  *log = "[DynamixelDriver] Failed to reboot!";
  return false;
}

bool DynamixelDriver::reset(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint32_t new_baud_rate = 0;
  uint8_t new_id = 1;

  const char* model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  uint16_t model_number = getModelNumber(id, log);
  if (model_number == 0) return false;

  if (getProtocolVersion() == 1.0)
  {

    if (!strncmp(model_name, "AX", strlen("AX")) ||
        !strncmp(model_name, "MX-12W", strlen("MX-12W")))
      new_baud_rate = 1000000;
    else
      new_baud_rate = 57600;

    sdk_error.dxl_comm_result = packetHandler_1->factoryReset(portHandler_, id, 0x00, &sdk_error.dxl_error);
    wait(2000);

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      result = setBaudrate(new_baud_rate, log);
      if (result == false) 
        return false;
      else
      {
        if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
            !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
            !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
            !strncmp(model_name, "XL", strlen("XL")) ||
            !strncmp(model_name, "XM", strlen("XM")) ||
            !strncmp(model_name, "XH", strlen("XH")) ||
            !strncmp(model_name, "PRO", strlen("PRO")))
        {
          result = setPacketHandler(2.0f, log);
          if (result == false) return false;
        }          
        else
        {
          result = setPacketHandler(1.0f, log);
          if (result == false) return false; 
        }
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }
  else if (getProtocolVersion() == 2.0)
  {
    sdk_error.dxl_comm_result = packetHandler_2->factoryReset(portHandler_, id, 0x0f, &sdk_error.dxl_error);
    wait(2000);

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320"))) 
        new_baud_rate = 1000000;
      else 
        new_baud_rate = 57600;

      result = setBaudrate(new_baud_rate, log);
      if (result == false) 
        return false;
      else
      {
        result = setPacketHandler(2.0f, log);
        if (result == false)  return false;
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to reset!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t address, uint8_t length, uint8_t* data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        address, 
                                                        length, 
                                                        data, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint8_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[1] = { data };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint16_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          address, 
                                                          length, 
                                                          data);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint8_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint16_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  *log = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[1] = {0};

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       address,
                                                       length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = data_read[0];
    return true;
  }

  *log = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[1] = {0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = data_read[0];
    return true;
  }

  *log = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint16_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[2] = {0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEWORD(data_read[0], data_read[1]);
    return true;
  }

  *log = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[4] = {0, 0, 0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
    return true;
  }

  *log = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::addSyncWriteHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync write handler are added (MAX = 5)";
    return false;
  }

  syncWriteHandler_[sync_write_handler_cnt_].control_item = control_item;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            control_item->address,
                                                                                            control_item->data_length);

  sync_write_handler_cnt_++;
  return true;                                                            
}

// bool DynamixelDriver::syncWrite(const char *item_name, int32_t *data)
// {
//   bool dxl_addparam_result = false;
//   int dxl_comm_result = COMM_TX_FAIL;

//   uint8_t data_byte[4] = {0, };
//   uint8_t cnt = 0;

//   SyncWriteHandler swh;
//   bool swh_found = false;

//   for (int index = 0; index < sync_write_handler_cnt_; index++)
//   {
//     if (!strncmp(syncWriteHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       swh = syncWriteHandler_[index];
//       swh_found = true;
//       break;
//     }
//   }

//   if (!swh_found)
//   {
//     return false;
//   }

//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
//       data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
//       data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
//       data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

//       dxl_addparam_result = swh.groupSyncWrite->addParam(tools_[i].getID()[j], (uint8_t *)&data_byte);
//       if (dxl_addparam_result != true)
//       {
//         return false;
//       }

//       cnt++;
//     }
//   }

//   dxl_comm_result = swh.groupSyncWrite->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }
//   swh.groupSyncWrite->clearParam();
//   return true;
// }

// bool DynamixelDriver::syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data)
// {
//   bool dxl_addparam_result = false;
//   int dxl_comm_result = COMM_TX_FAIL;

//   uint8_t data_byte[4] = {0, };
//   uint8_t cnt = 0;

//   SyncWriteHandler swh;
//   bool swh_found = false;

//   for (int index = 0; index < sync_write_handler_cnt_; index++)
//   {
//     if (!strncmp(syncWriteHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       swh = syncWriteHandler_[index];
//       swh_found = true;
//       break;
//     }
//   }

//   if (!swh_found)
//   {
//     return false;
//   }
  
//   for (int i = 0; i < id_num; i++)
//   {
//     data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
//     data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
//     data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
//     data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

//     dxl_addparam_result = swh.groupSyncWrite->addParam(id[i], (uint8_t *)&data_byte);
//     if (dxl_addparam_result != true)
//     {
//       return false;
//     }
//     cnt++;
//   }

//   dxl_comm_result = swh.groupSyncWrite->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }
//   swh.groupSyncWrite->clearParam();
//   return true;
// }

bool DynamixelDriver::addSyncReadHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync read handler are added (MAX = 5)";
    return false;
  }

  syncReadHandler_[sync_read_handler_cnt_].control_item = control_item;

  syncReadHandler_[sync_read_handler_cnt_++].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          control_item->address,
                                                                                          control_item->data_length);

  sync_read_handler_cnt_++;
  return true;       
}

// bool DynamixelDriver::syncRead(const char *item_name, int32_t *data)
// {
//   int dxl_comm_result = COMM_RX_FAIL;
//   bool dxl_addparam_result = false;
//   bool dxl_getdata_result = false;

//   int index = 0;

//   SyncReadHandler srh;
//   bool srh_found = false;
  
//   for (int index = 0; index < sync_read_handler_cnt_; index++)
//   {
//     if (!strncmp(syncReadHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       srh = syncReadHandler_[index];
//       srh_found = true;
//       break; // Found it, don't need to continue search
//     }
//   }
//   if (!srh_found)
//   {
//     return false; // did not find item_name in list
//   }
//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       dxl_addparam_result = srh.groupSyncRead->addParam(tools_[i].getID()[j]);
//       if (dxl_addparam_result != true)
//         return false;
//     }
//   }

//   dxl_comm_result = srh.groupSyncRead->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       uint8_t id = tools_[i].getID()[j];

//       dxl_getdata_result = srh.groupSyncRead->isAvailable(id, srh.control_item->address, srh.control_item->data_length);
//       if (dxl_getdata_result)
//       {
//         data[index++] = srh.groupSyncRead->getData(id, srh.control_item->address, srh.control_item->data_length);
//       }
//       else
//       {
//         return false;
//       }
//     }
//   }

//   srh.groupSyncRead->clearParam();

//   return true;
// }

// void DynamixelDriver::initBulkWrite()
// {
//   groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_[0]);
// }

// bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
// {
//   bool dxl_addparam_result = false;
//   uint8_t data_byte[4] = {0, };

//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) return false;
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data));
//   data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data));
//   data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data));
//   data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data));

//   dxl_addparam_result = groupBulkWrite_->addParam(id, control_item->address, control_item->data_length, data_byte);
//   if (dxl_addparam_result != true)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::bulkWrite()
// {
//   int dxl_comm_result = COMM_TX_FAIL;

//   dxl_comm_result = groupBulkWrite_->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   groupBulkWrite_->clearParam();

//   return true;
// }

// void DynamixelDriver::initBulkRead()
// {
//   groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_[0]);
// }

// bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name)
// {
//   bool dxl_addparam_result = false;

//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) return false;
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   dxl_addparam_result = groupBulkRead_->addParam(id, control_item->address, control_item->data_length);
//   if (dxl_addparam_result != true)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::sendBulkReadPacket()
// {
//   int dxl_comm_result = COMM_RX_FAIL;

//   dxl_comm_result = groupBulkRead_->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::bulkRead(uint8_t id, const char *item_name, int32_t *data)
// {
//   bool dxl_getdata_result = false;
//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) 
//   {
//     return false;
//   }
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   dxl_getdata_result = groupBulkRead_->isAvailable(id, control_item->address, control_item->data_length);
//   if (dxl_getdata_result != true)
//   {
//     return false;
//   }

//   *data = groupBulkRead_->getData(id, control_item->address, control_item->data_length);

//   return true;
// }

// int32_t DynamixelDriver::convertRadian2Value(uint8_t id, float radian)
// {
//   int32_t value = 0;
//   uint8_t factor = getTool(id);

//   if (radian > 0)
//   {
//     value = (radian * (tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMaxRadian()) + tools_[factor].getValueOfZeroRadianPosition();
//   }
//   else if (radian < 0)
//   {
//     value = (radian * (tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMinRadian()) + tools_[factor].getValueOfZeroRadianPosition();
//   }
//   else
//   {
//     value = tools_[factor].getValueOfZeroRadianPosition();
//   }

//   return value;
// }

// float DynamixelDriver::convertValue2Radian(uint8_t id, int32_t value)
// {
//   float radian = 0.0;
//   uint8_t factor = getTool(id);
//   if (factor == 0) factor = 0;  // just use first one

//   if (value > tools_[factor].getValueOfZeroRadianPosition())
//   {
//     radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMaxRadian() / (float)(tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
//   }
//   else if (value < tools_[factor].getValueOfZeroRadianPosition())
//   {
//     radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMinRadian() / (float)(tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
//   }

//   return radian;
// }

// int32_t DynamixelDriver::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
// {
//   int32_t value = 0;
//   int32_t zero_position = (max_position + min_position)/2;

//   if (radian > 0)
//   {
//     value = (radian * (max_position - zero_position) / max_radian) + zero_position;
//   }
//   else if (radian < 0)
//   {
//     value = (radian * (min_position - zero_position) / min_radian) + zero_position;
//   }
//   else
//   {
//     value = zero_position;
//   }

//   return value;
// }

// float DynamixelDriver::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
// {
//   float radian = 0.0;
//   int32_t zero_position = (max_position + min_position)/2;

//   if (value > zero_position)
//   {
//     radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
//   }
//   else if (value < zero_position)
//   {
//     radian = (float)(value - zero_position) * min_radian / (float)(min_position - zero_position);
//   }

//   return radian;
// }

// int32_t DynamixelDriver::convertVelocity2Value(uint8_t id, float velocity)
// {
//   int32_t value = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   // value = velocity * tools_[factor].getVelocityToValueRatio();

//   return value;
// }

// float DynamixelDriver::convertValue2Velocity(uint8_t id, int32_t value)
// {
//   float velocity = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   // velocity = value / tools_[factor].getVelocityToValueRatio();

//   return velocity;
// }

// int16_t DynamixelDriver::convertTorque2Value(uint8_t id, float torque)
// {
//   int16_t value = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   value = torque * tools_[factor].getTorqueToCurrentValueRatio();

//   return value;
// }

// float DynamixelDriver::convertValue2Torque(uint8_t id, int16_t value)
// {
//   float torque = 0.0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   torque = value / tools_[factor].getTorqueToCurrentValueRatio();

//   return torque;
// }

void DynamixelDriver::wait(uint16_t msec)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(msec);
#else
    usleep(1000*msec);
#endif
}
