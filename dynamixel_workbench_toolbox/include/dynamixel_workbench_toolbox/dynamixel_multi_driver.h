/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehoon Lim (Darby) */
#ifndef DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H
#define DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H

#include "dynamixel_driver.h"

namespace dynamixel_multi_driver
{
class DynamixelMultiDriver : public dynamixel_driver::DynamixelDriver
{
 public:
  std::vector<dynamixel_tool::DynamixelTool *> multi_dynamixel_;
  std::map<std::string, std::vector<int64_t> *> read_value_;

 private:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWriteMovingSpeed_;
  dynamixel::GroupSyncWrite *groupSyncWriteCurrent_;
  dynamixel::GroupSyncWrite *groupSyncWriteTorque_;
  dynamixel::GroupSyncWrite *groupSyncWriteProfileVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWriteProfileAcceleration_;
  dynamixel::GroupSyncWrite *groupSyncWriteOperatingMode_;
  dynamixel::GroupSyncWrite *groupSyncWritePositionPGain_;
  dynamixel::GroupSyncWrite *groupSyncWritePositionIGain_;
  dynamixel::GroupSyncWrite *groupSyncWritePositionDGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocityPGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocityIGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteCurrentLimit_;
  dynamixel::GroupSyncWrite *groupSyncWriteAccelerationLimit_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocityLimit_;
  dynamixel::GroupSyncWrite *groupSyncWriteMaxPositionLimit_;
  dynamixel::GroupSyncWrite *groupSyncWriteMinPositionLimit_;
  /* Goal torque for MX64 */
  dynamixel::GroupSyncWrite *groupSyncWriteGoalTorque_;
  dynamixel::GroupSyncWrite *groupSyncWriteTorqueModeEnable_;
  dynamixel::GroupSyncWrite *groupSyncWritePGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteIGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteDGain_;
  dynamixel::GroupSyncWrite *groupSyncWriteCWLimit_;
  dynamixel::GroupSyncWrite *groupSyncWriteCCWLimit_;

  dynamixel::GroupSyncRead  *groupSyncReadPosition_;
  dynamixel::GroupSyncRead  *groupSyncReadCurrent_;
  dynamixel::GroupSyncRead  *groupSyncReadVelocity_;
  dynamixel::GroupSyncRead  *groupSyncReadLoad_;

 public:
  DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version);
  ~DynamixelMultiDriver();

  bool loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo *> dynamixel_info);
  bool initSyncWrite();
  bool initSyncRead();
  dynamixel::GroupSyncWrite* setSyncWrite(std::string addr_name);
  dynamixel::GroupSyncRead*  setSyncRead(std::string addr_name);

  bool readMultiRegister(std::string addr_name);

  bool syncWriteTorque(std::vector<uint8_t> &onoff);
  bool syncWritePosition(std::vector<uint32_t> pos);
  bool syncWriteVelocity(std::vector<int32_t> vel);
  bool syncWriteMovingSpeed(std::vector<uint16_t> spd);
  bool syncWriteCurrent(std::vector<int16_t> cur);
  bool syncWriteProfileVelocity(std::vector<uint32_t> vel);
  bool syncWriteProfileAcceleration(std::vector<uint32_t> acc);
  bool syncWriteOperatingMode(std::vector<uint8_t> mode);
  bool syncWritePositionPGain(std::vector<uint16_t> pGain);
  bool syncWritePositionIGain(std::vector<uint16_t> iGain);
  bool syncWritePositionDGain(std::vector<uint16_t> dGain);
  bool syncWriteVelocityPGain(std::vector<uint16_t> pGain);
  bool syncWriteVelocityIGain(std::vector<uint16_t> iGain);
  bool syncWriteCurrentLimit(std::vector<uint16_t> current);
  bool syncWriteAccelerationLimit(std::vector<uint32_t> acceleration);
  bool syncWriteVelocityLimit(std::vector<uint32_t> velocity);
  bool syncWriteMaxPositionLimit(std::vector<uint32_t> maxPosition);
  bool syncWriteMinPositionLimit(std::vector<uint32_t> minPosition);
  /* Goal torque for MX64 */
  bool syncWriteGoalTorque(std::vector<uint16_t> goalTorque);
  bool syncWriteTorqueModeEnable(std::vector<uint8_t> &onoff);
  bool syncWritePGain(std::vector<uint8_t> &pGain);
  bool syncWriteIGain(std::vector<uint8_t> &iGain);
  bool syncWriteDGain(std::vector<uint8_t> &dGain);
  bool syncWriteCWLimit(std::vector<uint16_t> &cwLimit);
  bool syncWriteCCWLimit(std::vector<uint16_t> &ccwLimit);
  
  bool syncReadPosition(std::vector<uint32_t> &pos);
  bool syncReadCurrent(std::vector<uint16_t> &cur);
  bool syncReadVelocity(std::vector<uint32_t> &vel);
  bool syncReadLoad(std::vector<uint16_t> &torque);
};
}

#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H
