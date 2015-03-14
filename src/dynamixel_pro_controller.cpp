/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>

#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

#include <ros/package.h>
#include <XmlRpcValue.h>

#include <dynamixel_pro_controller/dynamixel_pro_controller.h>

#include <dynamixel_pro_driver/dynamixel_pro_driver.h>

using namespace dynamixel_pro_controller;
using namespace std;

DynamixelProController::DynamixelProController()
{
    shutting_down = false;
    nh = new ros::NodeHandle("~");

    //load the file containing model info, we're not using the param server here
    string path = ros::package::getPath("dynamixel_pro_controller");
    path += "/config/motor_data.yaml";

    YAML::Node doc;

#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::LoadFile(path);
#else
    ifstream fin(path.c_str());
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    for (int i = 0; i < doc.size(); i++)
    {
        dynamixel_spec spec;

        // Load the basic specs of this motor type 
        doc[i]["name"] >> spec.name;
        doc[i]["model_number"] >> spec.model_number;
        doc[i]["cpr"]  >> spec.cpr;
        doc[i]["gear_reduction"]  >> spec.gear_reduction;

        model_number2specs[spec.model_number] = spec;
    }

    //load all the info from the param server, with defaults
    nh->param<double>("publish_rate", publish_rate, 50.5);
    nh->param<bool>("publish_velocities", publish_velocities, false);

    string device;
    int baudrate;
    nh->param<std::string>("device", device, "/dev/ttyUSB0");
    nh->param<int>("baudrate", baudrate, 1000000);
    stringstream ss;
    ss << baudrate;

    driver = new dynamixel_pro_driver::DynamixelProDriver(device, ss.str());

    int num_motors = 0;
    
    // read in the information regarding the servos that we're supposed to 
    // connect to 
    if (nh->hasParam("servos"))
    {
        XmlRpc::XmlRpcValue servos;
        nh->getParam("servos", servos);
        //If there is no servos array in the param server, return
        if (!servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Invalid/missing servo information on the param server");
            ROS_BREAK();
        }


        num_motors = servos.size();
        //For every servo, load and verify its information
        for (int i = 0; i < servos.size(); i++)
        {
            dynamixel_info info;

            if (!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
                ROS_BREAK();
            }

            if (!servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("Invalid/Missing id for servo index %d", i);
                ROS_BREAK();
            }
            else
            {
                //store the servo's ID
                info.id = static_cast<int>(servos[i]["id"]);
            }

            if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
                ROS_BREAK();
            }
            else
            {
                //store the servo's corresponding joint 
                info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);
            }
           
            //Ping the servo to make sure that we can actually connect to it 
            // and that it is alive and well on our bus 
            if (driver->ping(info.id))
            {
                bool success = true;
                success &= driver->getModelNumber(info.id, info.model_number);
                success &= driver->getModelInfo(info.id, info.model_info);
                success &= model_number2specs.find(info.model_number) != model_number2specs.end();
                //make sure that we are successfully able to pull all the model
                //information

                if (success)
                {
                    //set up the lookup tables that we'll use later in the code
                    //to look up how to operate each joint
                    info.cpr = model_number2specs[info.model_number].cpr;
                    info.gear_reduction = model_number2specs[info.model_number].gear_reduction;

                    joint2dynamixel[info.joint_name] = info;

                    //initialize the status struct with the starting/ default
                    //status
                    dynamixel_status status;
                    status.id = info.id;
                    status.mode = UNKOWN;
                    status.torque_enabled = false;

                    id2status[info.id] = status;
                }
                else
                {
                    ROS_ERROR("Failed to load model information for dynamixel id %d", info.id);
                    ROS_ERROR("Model Number: %d, Model Info: %d ", info.model_number, info.model_info);
                    if (model_number2specs.find(info.model_number) != model_number2specs.end())
                        ROS_ERROR("Info is in database");
                    else
                        ROS_ERROR("Info is not in database");

                }
            }
            else
                ROS_ERROR("Cannot ping dyanmixel id: %d", info.id);
        }
    }
    else
    {
        ROS_ERROR("No servos details loaded to param server");
        ROS_BREAK();
    }

    //advertise the sensor feedback topic 
    jointStatePublisher  = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
   
    //Start listening to command messages. There is a queue size of 1k so that
    //we don't accidentally miss commands that are sent to us in batches for
    //many joints at once. 
    jointStateSubscriber = nh->subscribe<sensor_msgs::JointState>("/joint_commands", 
        1000, &DynamixelProController::jointStateCallback, this);
}

DynamixelProController::~DynamixelProController()
{
    shutting_down = false;
    delete nh;

    ros::Duration(0.1).sleep();//just in case. This should be changed to
    //something a tad more deterministic than this
    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)    
    {
        driver->setTorqueEnabled(iter->second.id, 0);
    }
    delete driver;

}

void DynamixelProController::startBroadcastingJointStates()
{
    broadcastTimer = nh->createTimer(ros::Duration(1.0 / publish_rate), &DynamixelProController::publishJointStates, this);
    broadcastTimer.start();
}

void DynamixelProController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (shutting_down)
        return;

    bool has_pos = false, has_vel = false, has_torque = false;
    control_mode new_mode = UNKOWN;

    //figure out which value is going to be our setpoint
    if (msg->position.size() > 0)
        has_pos = true;
    if (msg->velocity.size() > 0)
        has_vel = true;
    else if (msg->effort.size() > 0) 
        has_torque = true; 

    //figure out which mode we are going to operate the servos in 
    if (has_pos)
        new_mode = POSITION_CONTROL;
    else if (has_vel)
        new_mode = VELOCITY_CONTROL;
    else if (has_torque) 
        new_mode = TORQUE_CONTROL;

    vector<int> ids, velocities, positions, torques;

    //actually send the commands to the joints
    for (int i = 0; i < msg->name.size(); i++)
    {
        //lookup the information for that particular joint to be able to control it
        string name = msg->name[i];
        dynamixel_info info = joint2dynamixel[name];
        dynamixel_status &status = id2status[info.id];

        //change to new mode if needed
        if(status.mode != new_mode && new_mode != UNKOWN)
        {
            if (status.torque_enabled)//you can't seem to change modes while the the servo is enabled
                driver->setTorqueEnabled(info.id, 0);
            status.torque_enabled = false;

            driver->setOperatingMode(info.id, new_mode);//the enum is set up to correspond to the operating modes
            status.mode = new_mode;
        }

        //enable torque if needed
        if (!status.torque_enabled)
            driver->setTorqueEnabled(info.id, 1);
        status.torque_enabled = true;

        //prepare data to be sent to the motor
        ids.push_back(info.id);

        if (has_pos)
        {
            const double ToTicks = info.cpr / 2.0;
            double rad_pos = msg->position[i];
            int pos = static_cast<int>(round((rad_pos / M_PI) * ToTicks));
            positions.push_back(pos);
        }
        if (has_vel)
        {
            double rad_s_vel = msg->velocity[i];
            int vel = static_cast<int>(rad_s_vel / 2.0 / M_PI * 60.0 * info.gear_reduction);
            velocities.push_back(vel);
        }
        if (has_torque)
        {
            //replace the below with proper code when you can figure out the units
            static bool first_run = true;
            if (first_run)
                ROS_WARN("Dynamixel pro controller torque control mode not implemented");
        }

        
    }

    //send the setpoints in monolithic packets to reduce bandwidth
    if (has_pos && has_vel)
    {
        //send both a position and a velocity limit
        vector< vector<int> > data;
        for (int i = 0; i < ids.size(); i++)
        {
            vector<int> temp;
            temp.push_back(ids[i]);//order matters here
            temp.push_back(positions[i]);
            temp.push_back(abs(velocities[i])); //velocity limits should always be positive 
            data.push_back(temp);
        }
        driver->setMultiPositionVelocity(data);
    }
    else
    {
        //send only a position setpoint
        if (has_pos)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(positions[i]);
                data.push_back(temp);
            }
            driver->setMultiPosition(data);
        }

        //send only a velocity setpoint
        if (has_vel)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(velocities[i]);
                data.push_back(temp);
            }
            driver->setMultiVelocity(data);
        }
    }

    if (has_torque)
    {
        //TODO add this functionality
    }
}

void DynamixelProController::publishJointStates(const ros::TimerEvent& e)
{
    //don't access the driver after its been cleaned up
    if (shutting_down)
        return;

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    //Iterate over all connected servos
    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)
    {
        string joint_name = iter->first;
        dynamixel_info info = iter->second;

        //get the position and conditionally the velocity and then publish them
        //under the joint name which we just looked up
        int position, velocity;
        if (driver->getPosition(info.id, position))
        {
            const double FromTicks = 1.0 / (static_cast<double>(info.cpr) / 2.0);
            double rad_pos = position * FromTicks * M_PI;
            msg.name.push_back(joint_name);
            msg.position.push_back(rad_pos);
            if (publish_velocities && driver->getVelocity(info.id, velocity))
            {
                double rad_vel = ((double) velocity) * 2.0 * M_PI / 60.0 / info.gear_reduction; 
                msg.velocity.push_back(rad_vel);
            }
        }
    }
    jointStatePublisher.publish(msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_pro_controller");
  DynamixelProController controller;
  controller.startBroadcastingJointStates();

  ros::spin(); //use a single threaded spinner as I'm pretty sure this code isn't thread safe. 
}









