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

#include "dynamixel_workbench_toolbox/dynamixel_multi_driver.h"

using namespace dynamixel_multi_driver;

DynamixelMultiDriver::DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version)
  :dynamixel_driver::DynamixelDriver(device_name, baud_rate, protocol_version)
{
  portHandler_   = dynamixel_driver::DynamixelDriver::portHandler_;
  packetHandler_ = dynamixel_driver::DynamixelDriver::packetHandler_;
}

DynamixelMultiDriver::~DynamixelMultiDriver()
{

}

bool DynamixelMultiDriver::loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info)
{
  uint8_t error = 0;

  for (std::vector<dynamixel_driver::DynamixelInfo>::size_type num = 0; num < dynamixel_info.size(); ++num)
  {
    if (packetHandler_->ping(portHandler_, dynamixel_info[num]->model_id, &dynamixel_info[num]->model_number, &error) == COMM_SUCCESS)
    {
      dynamixel_tool::DynamixelTool *dynamixel = new dynamixel_tool::DynamixelTool(dynamixel_info[num]->model_id, dynamixel_info[num]->model_number);
      multi_dynamixel_.push_back(dynamixel);

      dynamixel_info[num]->model_name = dynamixel->model_name_;
    }
    else
    {
      return false;
    }
  }

  return true;
}

dynamixel::GroupSyncWrite* DynamixelMultiDriver::setSyncWrite(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncWrite * groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncWrite;
}

dynamixel::GroupSyncRead* DynamixelMultiDriver::setSyncRead(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncRead* groupSyncRead = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncRead;
}

bool DynamixelMultiDriver::initSyncWrite()
{
  groupSyncWriteTorque_   = setSyncWrite("torque_enable");
  groupSyncWritePosition_ = setSyncWrite("goal_position");

  if (getProtocolVersion() == 2.0)
  {
    groupSyncWriteVelocity_ = setSyncWrite("goal_velocity");
    groupSyncWriteOperatingMode_ = setSyncWrite("operating_mode");
    groupSyncWritePositionPGain_ = setSyncWrite("position_p_gain");
    groupSyncWritePositionIGain_ = setSyncWrite("position_i_gain");
    groupSyncWritePositionDGain_ = setSyncWrite("position_d_gain");
    groupSyncWriteVelocityPGain_ = setSyncWrite("velocity_p_gain");
    groupSyncWriteVelocityIGain_ = setSyncWrite("velocity_i_gain");
    groupSyncWriteCurrentLimit_ = setSyncWrite("current_limit");
    groupSyncWriteAccelerationLimit_ = setSyncWrite("acceleration_limit");
    groupSyncWriteVelocityLimit_ = setSyncWrite("velocity_limit");
    groupSyncWriteMaxPositionLimit_ = setSyncWrite("max_position_limit");
    groupSyncWriteMinPositionLimit_ = setSyncWrite("min_position_limit");

    if (multi_dynamixel_[0]->model_name_.find("XM") != std::string::npos)
    {
      groupSyncWriteCurrent_ = setSyncWrite("goal_current");
    }

    if (!(multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
    {
      groupSyncWriteProfileVelocity_     = setSyncWrite("profile_velocity");
      groupSyncWriteProfileAcceleration_ = setSyncWrite("profile_acceleration");
    }
  }
  else
  {
    groupSyncWriteMovingSpeed_ = setSyncWrite("moving_speed");
    groupSyncWriteGoalTorque_ = setSyncWrite("goal_torque");
    groupSyncWriteTorqueModeEnable_ = setSyncWrite("torque_control_mode_enable");
    groupSyncWritePGain_ = setSyncWrite("p_gain");
    groupSyncWriteIGain_ = setSyncWrite("i_gain");
    groupSyncWriteDGain_ = setSyncWrite("d_gain");
    groupSyncWriteCWLimit_ = setSyncWrite("cw_angle_limit");
    groupSyncWriteCCWLimit_ = setSyncWrite("ccw_angle_limit");
  }

  return true;
}

bool DynamixelMultiDriver::initSyncRead()
{
  if (getProtocolVersion() == 2.0)
  {
      groupSyncReadPosition_ = setSyncRead("present_position");
      groupSyncReadCurrent_ = setSyncRead("present_current");
      groupSyncReadVelocity_ = setSyncRead("present_velocity");
  }
  else
  {
      groupSyncReadPosition_ = setSyncRead("present_position");
      groupSyncReadVelocity_ = setSyncRead("present_speed");
      groupSyncReadLoad_ = setSyncRead("present_load");
  }
  return true;
}

bool DynamixelMultiDriver::readMultiRegister(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;
  int32_t value;

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dynamixel_= multi_dynamixel_[num];

    if (readRegister(addr_name, &value))
      read_data->push_back(value);
    else
      return false;
  }

  read_value_[addr_name] = read_data;

  return true;
}

bool DynamixelMultiDriver::syncWritePosition(std::vector<uint32_t> pos)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos[num]));

    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePosition_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteOperatingMode(std::vector<uint8_t> mode)
{
    bool dynamixel_addparam_result_;
    int8_t dynamixel_comm_result_;
    //uint8_t param_goal_position[4];
    uint8_t param_goal_position;

    //ROS_INFO( "Dynamixel ID lists :%d and: %d  mode parameters :%d and: %d" ,multi_dynamixel_[0]->id_ , multi_dynamixel_[1]->id_ , mode.at(0), mode.at(1));
    for(std::vector<dynamixel_tool::DynamixelTool *>::size_type num=0; num < 1; num++)
    {
        //param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(mode[num]));
        //param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(mode[num]));
        //param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(mode[num]));
        //param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(mode[num]));
        param_goal_position = mode[num];
        ROS_INFO("param to be written is : %d",param_goal_position);
        dynamixel_addparam_result_ = groupSyncWriteOperatingMode_->addParam(multi_dynamixel_[num]->id_, &param_goal_position);
        if(dynamixel_addparam_result_ != true)
        {
            ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
            //ROS_ERROR("Mode value: %d", param_goal_position[0]);
            return false;
        }
        ROS_INFO("Add param function is succesfull");
        dynamixel_comm_result_ = groupSyncWriteOperatingMode_->txPacket();
        if(dynamixel_comm_result_ != COMM_SUCCESS)
        {
            ROS_INFO("dynamixel_comm_result %d" , dynamixel_comm_result_);
            packetHandler_->printTxRxResult(dynamixel_comm_result_);
            return false;
        }
        groupSyncWriteOperatingMode_->clearParam();
        return true;
    }
}

bool DynamixelMultiDriver::syncWriteVelocity(std::vector<int32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteMovingSpeed(std::vector<uint16_t> spd)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_speed[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_speed[0] = DXL_LOBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[1] = DXL_HIBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[2] = DXL_LOBYTE(DXL_HIWORD(spd[num]));
    param_goal_speed[3] = DXL_HIBYTE(DXL_HIWORD(spd[num]));

    dynamixel_addparam_result_ = groupSyncWriteMovingSpeed_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_speed);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMovingSpeed_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMovingSpeed_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteCurrent(std::vector<int16_t> cur)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_current[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD(cur[num]));
    param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD(cur[num]));

    dynamixel_addparam_result_ = groupSyncWriteCurrent_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_current);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCurrent_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCurrent_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteGoalTorque(std::vector<uint16_t> goalTorque)
{
    bool dynamixel_addparam_result_;
    int8_t dynamixel_comm_result_;
    uint8_t param_goal_torque[4];

    for(std::vector<dynamixel_tool::DynamixelTool *>::size_type num=0; num < multi_dynamixel_.size(); ++num)
    {
        param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(goalTorque[num]));
        param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(goalTorque[num]));
        param_goal_torque[2] = DXL_LOBYTE(DXL_HIWORD(goalTorque[num]));
        param_goal_torque[3] = DXL_HIBYTE(DXL_HIWORD(goalTorque[num]));

        dynamixel_addparam_result_ = groupSyncWriteGoalTorque_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_torque);
        if(dynamixel_addparam_result_ != true)
        {
            ROS_ERROR("[ID:%03d] groupSyncWriteGoalTorque_ addParam failed", multi_dynamixel_[num]->id_);
            return false;
        }

    dynamixel_comm_result_ = groupSyncWriteGoalTorque_->txPacket();
    if(dynamixel_comm_result_ != COMM_SUCCESS)
    {
        packetHandler_->printTxRxResult(dynamixel_comm_result_);
        return false;
    }
    groupSyncWriteGoalTorque_->clearParam();
    return true;
}
}

bool DynamixelMultiDriver::syncWriteTorqueModeEnable(std::vector<uint8_t> &onoff)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_torque[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff[num]));
    param_goal_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff[num]));

    dynamixel_addparam_result_ = groupSyncWriteTorqueModeEnable_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_torque);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWriteTorqueModeEnable_ addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteTorqueModeEnable_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteTorqueModeEnable_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteTorque(std::vector<uint8_t> &onoff)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_torque[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff[num]));
    param_goal_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff[num]));

    dynamixel_addparam_result_ = groupSyncWriteTorque_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_torque);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteTorque_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteTorque_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteProfileVelocity(std::vector<uint32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileVelocity_->clearParam();
  return true;
}

 bool DynamixelMultiDriver::syncWriteProfileAcceleration(std::vector<uint32_t> acc)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_acceleration[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_acceleration[0] = DXL_LOBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[1] = DXL_HIBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[2] = DXL_LOBYTE(DXL_HIWORD(acc[num]));
    param_goal_profile_acceleration[3] = DXL_HIBYTE(DXL_HIWORD(acc[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileAcceleration_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_acceleration);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileAcceleration_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileAcceleration_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWritePositionPGain(std::vector<uint16_t> pGain)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_position_pgain[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_position_pgain[0] = DXL_LOBYTE(DXL_LOWORD(pGain[num]));
    param_position_pgain[1] = DXL_HIBYTE(DXL_LOWORD(pGain[num]));
    param_position_pgain[2] = DXL_LOBYTE(DXL_HIWORD(pGain[num]));
    param_position_pgain[3] = DXL_HIBYTE(DXL_HIWORD(pGain[num]));

    dynamixel_addparam_result_ = groupSyncWritePositionPGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_position_pgain);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePositionPGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePositionPGain_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWritePositionIGain(std::vector<uint16_t> iGain)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_position_igain[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_position_igain[0] = DXL_LOBYTE(DXL_LOWORD(iGain[num]));
    param_position_igain[1] = DXL_HIBYTE(DXL_LOWORD(iGain[num]));
    param_position_igain[2] = DXL_LOBYTE(DXL_HIWORD(iGain[num]));
    param_position_igain[3] = DXL_HIBYTE(DXL_HIWORD(iGain[num]));

    dynamixel_addparam_result_ = groupSyncWritePositionIGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_position_igain);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePositionIGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePositionIGain_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWritePositionDGain(std::vector<uint16_t> dGain)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_position_dgain[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_position_dgain[0] = DXL_LOBYTE(DXL_LOWORD(dGain[num]));
    param_position_dgain[1] = DXL_HIBYTE(DXL_LOWORD(dGain[num]));
    param_position_dgain[2] = DXL_LOBYTE(DXL_HIWORD(dGain[num]));
    param_position_dgain[3] = DXL_HIBYTE(DXL_HIWORD(dGain[num]));

    dynamixel_addparam_result_ = groupSyncWritePositionDGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_position_dgain);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePositionDGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePositionDGain_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteVelocityPGain(std::vector<uint16_t> pGain)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_velocity_pgain[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_velocity_pgain[0] = DXL_LOBYTE(DXL_LOWORD(pGain[num]));
    param_velocity_pgain[1] = DXL_HIBYTE(DXL_LOWORD(pGain[num]));
    param_velocity_pgain[2] = DXL_LOBYTE(DXL_HIWORD(pGain[num]));
    param_velocity_pgain[3] = DXL_HIBYTE(DXL_HIWORD(pGain[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocityPGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_velocity_pgain);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocityPGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocityPGain_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteVelocityIGain(std::vector<uint16_t> iGain)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_velocity_igain[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_velocity_igain[0] = DXL_LOBYTE(DXL_LOWORD(iGain[num]));
    param_velocity_igain[1] = DXL_HIBYTE(DXL_LOWORD(iGain[num]));
    param_velocity_igain[2] = DXL_LOBYTE(DXL_HIWORD(iGain[num]));
    param_velocity_igain[3] = DXL_HIBYTE(DXL_HIWORD(iGain[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocityIGain_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_velocity_igain);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocityIGain_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocityIGain_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWritePGain(std::vector<uint8_t> &pGain)
{
    bool dynamixel_addparam_result_;
    int8_t dynamixel_comm_result_;
    uint8_t param_pgain;
    for(std::vector<dynamixel_tool::DynamixelTool*>::size_type num=0; num < multi_dynamixel_.size(); ++num)
    {
        param_pgain = pGain[num];

        dynamixel_addparam_result_ = groupSyncWritePGain_->addParam(multi_dynamixel_[num]->id_, &param_pgain);
        if(dynamixel_addparam_result_ != true)
        {
            ROS_ERROR("[ID:%03d] groupSyncWrite addParam failed", multi_dynamixel_[num]->id_);
            return false;
        }
    }

    dynamixel_comm_result_ = groupSyncWritePGain_->txPacket();
    if(dynamixel_comm_result_ != COMM_SUCCESS)
    {
        packetHandler_->printTxRxResult(dynamixel_comm_result_);
        return false;
    }
    groupSyncWritePGain_->clearParam();
    return true;
}

bool DynamixelMultiDriver::syncWriteIGain(std::vector<uint8_t> &iGain)
{
    bool dynamixel_addparam_result_;
    int8_t dynamixel_comm_result_;
    uint8_t param_igain;
    for(std::vector<dynamixel_tool::DynamixelTool*>::size_type num=0; num < multi_dynamixel_.size(); ++num)
    {
        param_igain = iGain[num];

        dynamixel_addparam_result_ = groupSyncWriteIGain_->addParam(multi_dynamixel_[num]->id_, &param_igain);
        if(dynamixel_addparam_result_ != true)
        {
            ROS_ERROR("[ID:%03d] groupSyncWrite addParam failed", multi_dynamixel_[num]->id_);
            return false;
        }
    }

    dynamixel_comm_result_ = groupSyncWriteIGain_->txPacket();
    if(dynamixel_comm_result_ != COMM_SUCCESS)
    {
        packetHandler_->printTxRxResult(dynamixel_comm_result_);
        return false;
    }
    groupSyncWriteIGain_->clearParam();
    return true;
}

bool DynamixelMultiDriver::syncWriteDGain(std::vector<uint8_t> &dGain)
{
    bool dynamixel_addparam_result_;
    int8_t dynamixel_comm_result_;
    uint8_t param_dgain;
    for(std::vector<dynamixel_tool::DynamixelTool*>::size_type num=0; num < multi_dynamixel_.size(); ++num)
    {
        param_dgain = dGain[num];

        dynamixel_addparam_result_ = groupSyncWriteDGain_->addParam(multi_dynamixel_[num]->id_, &param_dgain);
        if(dynamixel_addparam_result_ != true)
        {
            ROS_ERROR("[ID:%03d] groupSyncWrite addParam failed", multi_dynamixel_[num]->id_);
            return false;
        }
    }

    dynamixel_comm_result_ = groupSyncWriteDGain_->txPacket();
    if(dynamixel_comm_result_ != COMM_SUCCESS)
    {
        packetHandler_->printTxRxResult(dynamixel_comm_result_);
        return false;
    }
    groupSyncWriteDGain_->clearParam();
    return true;
}

bool DynamixelMultiDriver::syncWriteCWLimit(std::vector<uint16_t> &cwLimit)
{
  bool dynamixel_add_param_result_;
  int8_t dynamixel_comm_result_;
  uint8_t param_cwLimit_[2];
  for(std::vector<dynamixel_tool::DynamixelTool*>::size_type num=0; num < multi_dynamixel_.size(); ++num)
  {
    param_cwLimit_[0] = DXL_LOBYTE(DXL_LOWORD(cwLimit[num]));
    param_cwLimit_[1] = DXL_HIBYTE(DXL_LOWORD(cwLimit[num]));

    dynamixel_add_param_result_ = groupSyncWriteCWLimit_->addParam(multi_dynamixel_[num]->id_, param_cwLimit_);
    if(dynamixel_add_param_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWriteCWLimit_ addParam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCWLimit_->txPacket();
  if(dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCWLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteCCWLimit(std::vector<uint16_t> &ccwLimit)
{
  bool dynamixel_add_param_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_ccwLimit_[2];
  for(std::vector<dynamixel_tool::DynamixelTool*>::size_type num=0; num < multi_dynamixel_.size(); ++num)
  {
    param_ccwLimit_[0] = DXL_LOBYTE(DXL_LOWORD(ccwLimit[num]));
    param_ccwLimit_[1] = DXL_HIBYTE(DXL_LOWORD(ccwLimit[num]));

    dynamixel_add_param_result_ = groupSyncWriteCCWLimit_->addParam(multi_dynamixel_[num]->id_, param_ccwLimit_);
    if(dynamixel_add_param_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWriteCCWLimit_ addParam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCCWLimit_->txPacket();
  if(dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCCWLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteCurrentLimit(std::vector<uint16_t> current)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_current_limit[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_current_limit[0] = DXL_LOBYTE(DXL_LOWORD(current[num]));
    param_current_limit[1] = DXL_HIBYTE(DXL_LOWORD(current[num]));
    param_current_limit[2] = DXL_LOBYTE(DXL_HIWORD(current[num]));
    param_current_limit[3] = DXL_HIBYTE(DXL_HIWORD(current[num]));

    dynamixel_addparam_result_ = groupSyncWriteCurrentLimit_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_current_limit);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCurrentLimit_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCurrentLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteAccelerationLimit(std::vector<uint32_t> acceleration)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_acceleration_limit[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_acceleration_limit[0] = DXL_LOBYTE(DXL_LOWORD(acceleration[num]));
    param_acceleration_limit[1] = DXL_HIBYTE(DXL_LOWORD(acceleration[num]));
    param_acceleration_limit[2] = DXL_LOBYTE(DXL_HIWORD(acceleration[num]));
    param_acceleration_limit[3] = DXL_HIBYTE(DXL_HIWORD(acceleration[num]));

    dynamixel_addparam_result_ = groupSyncWriteAccelerationLimit_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_acceleration_limit);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteAccelerationLimit_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteAccelerationLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteVelocityLimit(std::vector<uint32_t> velocity)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_velocity_limit[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_velocity_limit[0] = DXL_LOBYTE(DXL_LOWORD(velocity[num]));
    param_velocity_limit[1] = DXL_HIBYTE(DXL_LOWORD(velocity[num]));
    param_velocity_limit[2] = DXL_LOBYTE(DXL_HIWORD(velocity[num]));
    param_velocity_limit[3] = DXL_HIBYTE(DXL_HIWORD(velocity[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocityLimit_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_velocity_limit);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocityLimit_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocityLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteMaxPositionLimit(std::vector<uint32_t> maxPosition)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_maxposition_limit[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_maxposition_limit[0] = DXL_LOBYTE(DXL_LOWORD(maxPosition[num]));
    param_maxposition_limit[1] = DXL_HIBYTE(DXL_LOWORD(maxPosition[num]));
    param_maxposition_limit[2] = DXL_LOBYTE(DXL_HIWORD(maxPosition[num]));
    param_maxposition_limit[3] = DXL_HIBYTE(DXL_HIWORD(maxPosition[num]));

    dynamixel_addparam_result_ = groupSyncWriteMaxPositionLimit_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_maxposition_limit);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMaxPositionLimit_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMaxPositionLimit_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteMinPositionLimit(std::vector<uint32_t> minPosition)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_minposition_limit[4];
  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_minposition_limit[0] = DXL_LOBYTE(DXL_LOWORD(minPosition[num]));
    param_minposition_limit[1] = DXL_HIBYTE(DXL_LOWORD(minPosition[num]));
    param_minposition_limit[2] = DXL_LOBYTE(DXL_HIWORD(minPosition[num]));
    param_minposition_limit[3] = DXL_HIBYTE(DXL_HIWORD(minPosition[num]));

    dynamixel_addparam_result_ = groupSyncWriteMinPositionLimit_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_minposition_limit);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMinPositionLimit_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMinPositionLimit_->clearParam();
  return true;
}


bool DynamixelMultiDriver::syncReadPosition(std::vector<uint32_t> &pos)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t position;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_position"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  pos.clear();

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadPosition_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      position  = groupSyncReadPosition_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      pos.push_back(position);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadPosition_->clearParam();

  return true;
}

bool DynamixelMultiDriver::syncReadCurrent(std::vector<uint16_t> &cur)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint16_t current;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_current"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  cur.clear();

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadCurrent_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadCurrent_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadCurrent_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      current  = groupSyncReadCurrent_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      cur.push_back(current);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadCurrent_->clearParam();

  return true;
}

bool DynamixelMultiDriver::syncReadVelocity(std::vector<uint32_t> &vel)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t velocity;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_current"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  vel.clear();

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadVelocity_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadVelocity_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadVelocity_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      velocity  = groupSyncReadVelocity_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      vel.push_back(velocity);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadVelocity_->clearParam();

  return true;
}
