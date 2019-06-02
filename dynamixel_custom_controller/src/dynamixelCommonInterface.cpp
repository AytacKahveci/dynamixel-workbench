/* @author Aytaç Kahveci */
#include <dynamixel_custom_controller/dynamixelCommonInterface.h>
#include <unistd.h>
#include <stdlib.h>

namespace dynamixel_common_interface
{

    DynamixelCommonInterface::DynamixelCommonInterface(ros::NodeHandle& nh)
    {
        using namespace hardware_interface;
        registerInterface(&jnt_poseff_interface);
        registerInterface(&jnt_state_interface);

        node_ns_ = nh.getUnresolvedNamespace();
        std::string param_name = node_ns_ + "/motors";
        std::vector<std::string> motorNames;
        if(!nh.getParam(param_name, motorNames))
        {
            ROS_ERROR("Unable to find motor names in the param server %s", param_name.c_str());
        }

        int limit_size = motorNames.size() * 2;
        limits_ = new int[limit_size];

        for(int i=0; i<motorNames.size(); i++)
        {
            ROS_INFO("Motor name%d: %s", i, motorNames[i].c_str());

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/id";
            int temp;

            if(!nh.getParam(param_name, temp))
            {
                ROS_ERROR("Unable to find motor ids in the param server %s", param_name.c_str());
            }
            motorId.push_back((int8_t)temp);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/operating_mode";
            if(!nh.getParam(param_name, temp))
            {
                temp = 3;
                ROS_WARN("Unable to find operatingMode in the param server %s and default val %d is assigned", param_name.c_str(), temp);

            }
            operatingMode.push_back((uint8_t)temp);

            int tempGains;
            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/gains/position_kp";
            if(!nh.getParam(param_name, tempGains))
            {
                tempGains = 800; //default positionKp gain
                ROS_WARN("Unable to find position_kp val in the param server %s and default val %d is assigned", param_name.c_str(), tempGains);
            }
            positionKp.push_back((uint16_t)tempGains);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/gains/position_ki";
            if(!nh.getParam(param_name, tempGains))
            {
                tempGains = 0; //default positionKi gain
                ROS_WARN("Unable to find position_ki val in the param server %s and default val %d is assigned", param_name.c_str(), tempGains);
            }
            positionKi.push_back((uint16_t)tempGains);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/gains/position_kd";
            if(!nh.getParam(param_name, tempGains))
            {
                tempGains = 0; //default positionKd gain
                ROS_WARN("Unable to find position_kd val in the param server %s and default val %d is assigned", param_name.c_str(), tempGains);
            }
            positionKd.push_back((uint16_t)tempGains);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/gains/velocity_ki";
            if(!nh.getParam(param_name, tempGains))
            {
                tempGains = 1920; //default velocityKi gain
                ROS_WARN("Unable to find velocity_ki val in the param server %s and default val %d is assigned", param_name.c_str(), tempGains);
            }
            velocityKi.push_back((uint16_t)tempGains);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/gains/velocity_kp";
            if(!nh.getParam(param_name, tempGains))
            {
                tempGains = 100; //default velocityKp gain
                ROS_WARN("Unable to find velocity_kp val in the param server %s and default val %d is assigned", param_name.c_str(), tempGains);
            }
            velocityKp.push_back((uint16_t)tempGains);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/current_limit";
            int tempLimit;
            if(!nh.getParam(param_name, tempLimit))
            {
                tempLimit = 1193; //default currentLimit gain
                ROS_WARN("Unable to find current_limit val in the param server %s and default val %d is assigned", param_name.c_str(), tempLimit);
            }
            currentLimit.push_back((uint16_t)tempLimit);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/acceleration_limit";
            if(!nh.getParam(param_name, tempLimit))
            {
                tempLimit = 32767; //default accelerationLimit gain
                ROS_WARN("Unable to find acceleration_limit val in the param server %s and default val %d is assigned", param_name.c_str(), tempLimit);
            }
            accelerationLimit.push_back((uint32_t)tempLimit);

            param_name = node_ns_ + "/motor_params/" + motorNames[i] + "/velocity_limit";
            if(!nh.getParam(param_name, tempLimit))
            {
                tempLimit = 350; //default velocity_limit gain
                ROS_WARN("Unable to find acceleration_limit val in the param server %s and default val %d is assigned", param_name.c_str(), tempLimit);
            }
            velocityLimit.push_back((uint32_t)tempLimit);

            /* Joint Limits */
            param_name = "/" + motorNames[i] + "/forward_limit";
            if(!nh.getParam(param_name, limits_[i*2]))
            {
                ROS_ERROR("Unable to find param name %s", param_name.c_str());
                exit(EXIT_FAILURE);
            }
            param_name = "/" + motorNames[i] + "/backward_limit";
            if(!nh.getParam(param_name, limits_[i*2 + 1]))
            {
                ROS_ERROR("Unable to find param name %s", param_name.c_str());
                exit(EXIT_FAILURE);
            }
            
            ROS_INFO("%s/Id %d",motorNames[i].c_str(), motorId[i]);
            ROS_INFO("%s/operatingMode %d",motorNames[i].c_str(), operatingMode[i]);
            ROS_INFO("%s/position_kp %d",motorNames[i].c_str(), positionKp[i]);
            ROS_INFO("%s/position_ki %d",motorNames[i].c_str(), positionKi[i]);
            ROS_INFO("%s/position_kd %d",motorNames[i].c_str(), positionKd[i]);
            ROS_INFO("%s/velocity_kp %d",motorNames[i].c_str(), velocityKp[i]);
            ROS_INFO("%s/velocity_ki %d",motorNames[i].c_str(), velocityKi[i]);
            ROS_INFO("%s/current_limit %d",motorNames[i].c_str(), currentLimit[i]);
            ROS_INFO("%s/acceleration_limit %d",motorNames[i].c_str(), accelerationLimit[i]);
            ROS_INFO("%s/velocity_limit %d",motorNames[i].c_str(), velocityLimit[i]);
            ROS_INFO("%s/lower_limit %d",motorNames[i].c_str(), limits_[i*2]);
            ROS_INFO("%s/upper_limit %d",motorNames[i].c_str(), limits_[i*2 + 1]);            
            ROS_INFO("*****************************");
        }

        motorNumbers = motorId.size();

        param_name = node_ns_ + "/port";
        if(!nh.getParam(param_name, port))
        {
            ROS_ERROR("Unable to find port in the param server %s", param_name.c_str());
        }

        param_name = node_ns_ + "/baudrate";
        if(!nh.getParam(param_name, baudrate))
        {
            baudrate = 1000000; // 1000000 bps
            ROS_ERROR("Unable to find baudrate in the param server %s and default val %d is assigned", param_name.c_str(), baudrate);
        }

        param_name = node_ns_ + "/protocol_version";
        if(!nh.getParam(param_name, protocolVersion))
        {
            protocolVersion = 2.0;
            ROS_WARN("Unable to find protocolVersion in the param server %s and default val %f is assigned", param_name.c_str(), protocolVersion);
        }

        ROS_INFO("port: %s", port.c_str());
        ROS_INFO("baudrate: %d", baudrate);
        ROS_INFO("protocol_version: %f", protocolVersion);
        ROS_INFO("*****************************");

        
        param_name = node_ns_ + "/joints";
        if(!nh.getParam(param_name, joint_names_))
        {
            ROS_ERROR("Unable to find joint_states in the param server %s", param_name.c_str());
        }
        jointNumbers = joint_names_.size();
        ROS_INFO("joint1: %s, joint2: %s", joint_names_[0].c_str(), joint_names_[1].c_str());
        if(jointNumbers == 0)
    	{
    		ROS_ERROR_STREAM("List of joint names is empty.");
    	}

        if(motorNumbers != jointNumbers)
        {
            ROS_ERROR("Joint Numbers and Motor Numbers are not equal in the param server %s", param_name.c_str());
        }

        if(loadDynamixel())
        {
            checkLoadDynamixel();
        }
        else
        {
            ROS_ERROR("Can't Load Dynamixels, Please Check Parameters");
        }


        if (!multi_driver_->initSyncWrite())
    	  ROS_ERROR("Init SyncWrite Failed!");

    	if (!multi_driver_->initSyncRead())
    	  ROS_ERROR("Init SyncRead Failed!");

        if(!multi_driver_->syncWritePositionPGain(positionKp))
          ROS_ERROR("Error updating position_kp register");

        if(!multi_driver_->syncWritePositionIGain(positionKi))
          ROS_ERROR("Error updating position_ki register");

        if(!multi_driver_->syncWritePositionDGain(positionKd))
          ROS_ERROR("Error updating position_kd register");

        if(!multi_driver_->syncWriteVelocityPGain(velocityKp))
          ROS_ERROR("Error updating velocity_kp register");

        if(!multi_driver_->syncWriteVelocityIGain(velocityKi))
          ROS_ERROR("Error updating velocity_ki register");

        if(!multi_driver_->syncWriteCurrentLimit(currentLimit))
          ROS_ERROR("Error updating current_limit register");

        if(!multi_driver_->syncWriteAccelerationLimit(accelerationLimit))
          ROS_ERROR("Error updating accelerationLimit register");

        if(!multi_driver_->syncWriteAccelerationLimit(velocityLimit))
          ROS_ERROR("Error updating velocityLimit register");

        vel_=new double[motorNumbers];
      	pos_=new double[motorNumbers];
      	eff_=new double[motorNumbers];
      	cmd_pos_=new double[motorNumbers];
      	cmd_eff_=new double[motorNumbers];
      	last_cmd_pos_=new double[motorNumbers];
      	last_cmd_eff_=new double[motorNumbers];

        writeValue_ = new WriteValue;
    	motorPos_ = new MotorPos;

    	motorPos_->des_pos.clear();


        if(!setTorque(true))
        {
            ROS_ERROR("Could not set torque");
        }
        usleep(2000000);
        multi_driver_->readMultiRegister("present_position");
    	for(int i=0; i < motorNumbers; i++)
    	{
    		vel_[i]=0;
        	pos_[i]=0;
        	eff_[i]=0;
        	cmd_pos_[i] = (double)multi_driver_->read_value_["present_position"]->at(i);
        	last_cmd_pos_[i]= cmd_pos_[i];
        	last_cmd_eff_[i]= eff_[i];
    	}

        for(int i=0; i<motorNumbers; i++)
        {
            try{
                hardware_interface::JointStateHandle state_handle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
                jnt_state_interface.registerHandle(state_handle);

                hardware_interface::PosEffJointHandle joint_handle_pos(jnt_state_interface.getHandle(joint_names_[i]), &cmd_pos_[i], &cmd_eff_[i]);
                jnt_poseff_interface.registerHandle(joint_handle_pos);
            }
            catch(...){
                ROS_ERROR("Exception thrown!!");
            }
        }


    }

    DynamixelCommonInterface::~DynamixelCommonInterface(){
    	setTorque(false);
    	delete pos_;
        delete vel_;
        delete eff_;
        delete cmd_pos_;
        delete cmd_eff_;
        delete last_cmd_pos_;
        delete last_cmd_eff_;
    	delete writeValue_;
    	delete motorPos_;
        delete limits_;
    }

    bool DynamixelCommonInterface::loadDynamixel()
    {
        bool ret = false;

        for(int i=0; i < motorNumbers; i++)
        {
            dynamixel_driver::DynamixelInfo* motorInfo = new dynamixel_driver::DynamixelInfo;
            motorInfo->model_id = motorId[i];
            motorInfo->lode_info.device_name = port;
            motorInfo->lode_info.baud_rate = baudrate;
            motorInfo->lode_info.protocol_version = protocolVersion;
            dynamixel_info_.push_back(motorInfo);
        }

        multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(port, baudrate, protocolVersion);

        ret = multi_driver_->loadDynamixel(dynamixel_info_);
        return ret;
    }

    void DynamixelCommonInterface::checkLoadDynamixel()
    {
        ROS_INFO("------------------------------------");
    	ROS_INFO("DynamixelCommonController");
    	ROS_INFO("------------------------------------");
    	ROS_INFO("Motors Info");
        //readDynamixelState();
        multi_driver_->readMultiRegister("operating_mode");
        multi_driver_->readMultiRegister("position_p_gain");
        multi_driver_->readMultiRegister("position_i_gain");
        multi_driver_->readMultiRegister("position_d_gain");
        multi_driver_->readMultiRegister("velocity_p_gain");
        multi_driver_->readMultiRegister("velocity_i_gain");
        multi_driver_->readMultiRegister("current_limit");
        multi_driver_->readMultiRegister("acceleration_limit");
        multi_driver_->readMultiRegister("velocity_limit");

        for(std::vector<dynamixel_tool::DynamixelTool*>::size_type i=0; i<dynamixel_info_.size(); i++)
        {
            ROS_INFO("ID : %d", dynamixel_info_[i]->model_id);
            ROS_INFO("MODEL : %s", dynamixel_info_[i]->model_name.c_str());
            ROS_INFO(" ");
        }

        for(int i=0; i<motorNumbers; i++)
        {
            ROS_INFO("Motor %d Operating Mode: %ld",i, multi_driver_->read_value_["operating_mode"]->at(i) );
            //ROS_INFO("Motor %d Present Position: %ld",i, multi_driver_->read_value_["present_position"]->at(i) );
            //ROS_INFO("Motor %d Goal Position: %ld",i, multi_driver_->read_value_["goal_position"]->at(i));
            ROS_INFO("Motor %d Position P Gain: %ld",i, multi_driver_->read_value_["position_p_gain"]->at(i) );
            ROS_INFO("Motor %d Position I Gain: %ld",i, multi_driver_->read_value_["position_i_gain"]->at(i) );
            ROS_INFO("Motor %d Position D Gain: %ld",i, multi_driver_->read_value_["position_d_gain"]->at(i) );
            ROS_INFO("Motor %d Velocity P Gain: %ld",i, multi_driver_->read_value_["velocity_p_gain"]->at(i) );
            ROS_INFO("Motor %d Velocity I Gain: %ld",i, multi_driver_->read_value_["velocity_i_gain"]->at(i) );
            ROS_INFO("Motor %d Current Limit: %ld",i, multi_driver_->read_value_["current_limit"]->at(i) );
            ROS_INFO("Motor %d Acceleration Limit: %ld",i, multi_driver_->read_value_["acceleration_limit"]->at(i) );
            ROS_INFO("Motor %d Velocity Limit: %ld",i, multi_driver_->read_value_["velocity_limit"]->at(i) );

        }
    }

    bool DynamixelCommonInterface::readDynamixelState()
    {
      multi_driver_->readMultiRegister("torque_enable");

      multi_driver_->readMultiRegister("present_position");

      multi_driver_->readMultiRegister("goal_position");
      multi_driver_->readMultiRegister("moving");

      if (multi_driver_->getProtocolVersion() == 2.0)
      {
        if (multi_driver_->multi_dynamixel_[motorNumbers]->model_name_.find("XM") != std::string::npos)
        {
          multi_driver_->readMultiRegister("goal_current");

          multi_driver_->readMultiRegister("present_current");
        }
        multi_driver_->readMultiRegister("goal_velocity");
        multi_driver_->readMultiRegister("present_velocity");
      }
      else
      {
        multi_driver_->readMultiRegister("moving_speed");
        multi_driver_->readMultiRegister("present_speed");
      }
    }

    bool DynamixelCommonInterface::setTorque(bool onoff){
    	writeValue_->torque.clear();
    	for(std::size_t i=0; i < motorNumbers; i++)
    	{
    		writeValue_->torque.push_back(onoff);
    	}

    	if(!multi_driver_->syncWriteTorque(writeValue_->torque))
    	{
    		ROS_ERROR("SyncWrite Torque Failed!");
    		return false;
    	}

    	return true;
    }

    bool DynamixelCommonInterface::setCurrent(double *currentArray)
    {
    	writeValue_->current.clear();
    	for(std::size_t i=0; i< motorNumbers; i++)
    	{
    		writeValue_->current.push_back((int16_t)currentArray[i]);
    	}

    	if(!multi_driver_->syncWriteCurrent(writeValue_->current))
    	{
    		ROS_ERROR("SyncWrite Current Failed");
    		return false;
    	}

    	return true;
    }

    bool DynamixelCommonInterface::setPosition(double *currentArray)
    {
      motorPos_->des_pos.clear();
      for(std::size_t i=0; i<motorNumbers; i++)
      {
    	  motorPos_->des_pos.push_back((uint32_t)currentArray[i]);
      }

      if (!multi_driver_->syncWritePosition(motorPos_->des_pos))
      {
        ROS_ERROR("SyncWrite Position Failed!");
        return false;
      }

      return true;
    }

    bool DynamixelCommonInterface::setOperatingMode(const std::vector<uint8_t> &mode)
    {
    	if(!multi_driver_->syncWriteOperatingMode(mode))
    	{
    		ROS_ERROR("syncWriteOperatingMode Failed!");
    		return false;
    	}
    	return true;
    }

    bool DynamixelCommonInterface::setVelocityProfile(std::vector<uint32_t> &vel)
    {
        if(!multi_driver_->syncWriteProfileVelocity(vel))
        {
            ROS_ERROR("syncWriteProfileVelocity Failed!");
    		return false;
        }
        return true;
    }

    bool DynamixelCommonInterface::setAccelerationProfile(std::vector<uint32_t> &acc)
    {
        if(!multi_driver_->syncWriteProfileAcceleration(acc))
        {
            ROS_ERROR("syncWriteProfileAcceleration Failed!");
    		return false;
        }
        return true;
    }

    void DynamixelCommonInterface::read()
    {
    	multi_driver_->readMultiRegister("present_position");
    	if (multi_driver_->getProtocolVersion() == 2.0)
        {
          if (multi_driver_->multi_dynamixel_[0]->model_name_.find("XM") != std::string::npos)
          {
            multi_driver_->readMultiRegister("present_current");
          }
        }
        else
        {
          //multi_driver_->readMultiRegister("moving_speed");
         // multi_driver_->readMultiRegister("present_speed");
        }

    	for(std::size_t i=0; i<motorNumbers; i++)
    	{
    		pos_[i] = (double)multi_driver_->read_value_["present_position"]->at(i);
    		//vel_[i] = (double)multi_driver_->read_value_["present_velocity"]->at(i);
    		eff_[i] = (double)multi_driver_->read_value_["present_current"]->at(i);
    		ROS_INFO_STREAM("POS" << i <<": "<< pos_[i] << " VEL" << i <<": " << vel_[i] <<" EFF" << i << ": " << eff_[i]);
    	}
    }

    void DynamixelCommonInterface::write()
    {
    	bool changes_pos=false;
        bool changes_eff=false;
    	for(std::size_t i=0; i<motorNumbers; i++)
    	{
            if((last_cmd_pos_[i] != cmd_pos_[i]))
    		{
    			last_cmd_pos_[i]= cmd_pos_[i];
    			changes_pos = true;
    		}

    		if(last_cmd_eff_[i] != cmd_eff_[i])
    		{
    			last_cmd_eff_[i] = cmd_eff_[i];
    			changes_eff = true;
    		}
    	}

            if(changes_pos)
        	{
                ROS_INFO("/lower_limit %d", limits_[0]);
                ROS_INFO("/upper_limit %d", limits_[1]);
                ROS_INFO("/lower_limit %d", limits_[2]);
                ROS_INFO("/upper_limit %d", limits_[3]);
    			if(cmd_pos_[0] <= limits_[1]) //forward_limit limit
    			{
    				cmd_pos_[0] = limits_[1] + 20;
    				ROS_ERROR("Forward Limit val is exceeded for MotorId0");
    			}
    			if(cmd_pos_[0] >= limits_[0]) //backward limit
    			{
    				cmd_pos_[0] = limits_[0] - 20;
    				ROS_ERROR("Backward Limit val is exceeded for MotorId0");
    			}

    			if(cmd_pos_[1] <= limits_[3]) //forward_limit limit
    			{
    				cmd_pos_[1] = limits_[3] + 20;
    				ROS_ERROR("Forward Limit val is exceeded for MotorId1");
    			}
    			if(cmd_pos_[1] >= limits_[2]) //backward limit
    			{
    				cmd_pos_[1] = limits_[2] - 20;
    				ROS_ERROR("Backward Limit val is exceeded for MotorId1");
    			}

        		setPosition(cmd_pos_);
        		ROS_INFO_STREAM("****POSSENDED ID1:"<< cmd_pos_[0] << ", ID2:" << cmd_pos_[1] << " ****");
        		ROS_INFO("SEND POS!");
        	}
    		if(changes_eff)
    		{
    			setCurrent(cmd_eff_);
    			ROS_INFO_STREAM("CURRENTSENDED "<< 0 << ":" << cmd_eff_[0]);
    			ROS_INFO("SEND CURRENT!");
    		}

    }


}
