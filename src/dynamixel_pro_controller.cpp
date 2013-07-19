#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "yaml-cpp/yaml.h"

#include <ros/package.h>
#include <XmlRpcValue.h>

#include <dynamixel_pro_controller/dynamixel_pro_controller.h>

#include <dynamixel_pro_driver/dynamixel_pro_driver.h>

using namespace dynamixel_pro_controller;
using namespace std;

DynamixelProController::DynamixelProController()
{
    nh = new ros::NodeHandle("~");

    //load the file containing model info
    string path = ros::package::getPath("dynamixel_pro_controller");
    path += "/config/motor_data.yaml";

    ifstream fin(path.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);
    for (int i = 0; i < doc.size(); i++)
    {
        dynamixel_spec spec;

        doc[i]["name"] >> spec.name;
        doc[i]["model_number"] >> spec.model_number;
        doc[i]["cpr"]  >> spec.cpr;
        doc[i]["gear_reduction"]  >> spec.gear_reduction;

        model_number2specs[spec.model_number] = spec;
    }

    //load all the info from the param server
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

    if (nh->hasParam("servos"))
    {
        XmlRpc::XmlRpcValue servos;
        nh->getParam("servos", servos);
        if (!servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Invalid/missing servo information on the param server");
            ROS_BREAK();
        }


        num_motors = servos.size();
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
                info.id = static_cast<int>(servos[i]["id"]);
            }

            if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
                ROS_BREAK();
            }
            else
            {
                info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);
            }
            
            if (driver->ping(info.id))
            {
                bool success = true;
                success &= driver->getModelNumber(info.id, info.model_number);
                success &= driver->getModelInfo(info.id, info.model_info);

                success &= model_number2specs.find(info.model_number) != model_number2specs.end();

                if (success)
                {
                    info.cpr = model_number2specs[info.model_number].cpr;
                    info.gear_reduction = model_number2specs[info.model_number].gear_reduction;

                    joint2dynamixel[info.joint_name] = info;

                    dynamixel_status status;
                    status.id = info.id;
                    status.mode = UNKOWN;
                    status.torque_enabled = false;

                    id2status[info.id] = status;
                }
                else
                {
                    ROS_ERROR("Failed to load model information for dynamixel id %d", info.id);
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

    jointStatePublisher  = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
    jointStateSubscriber = nh->subscribe<sensor_msgs::JointState>("/joint_commands", 1000, &DynamixelProController::jointStateCallback, this);// just so we don't skip servos when we get a bunch of command messages for different servos
}

DynamixelProController::~DynamixelProController()
{
}

void DynamixelProController::startListeningForCommands()
{
}

void DynamixelProController::startBroadcastingJointStates()
{
    broadcastTimer = nh->createTimer(ros::Duration(1.0 / publish_rate), &DynamixelProController::publishJointStates, this);
    broadcastTimer.start();
}

void DynamixelProController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    bool has_pos = false, has_vel = false, has_torque = false;
    control_mode new_mode = UNKOWN;

    if (msg->position.size() > 0)
        has_pos = true;
    if (msg->velocity.size() > 0)
        has_vel = true;
    else if (msg->effort.size() > 0) 
        has_torque = true; 

    if (has_pos)
        new_mode = POSITION_CONTROL;
    else if (has_vel)
        new_mode = VELOCITY_CONTROL;
    else if (has_torque) 
        new_mode = TORQUE_CONTROL;

    vector<int> ids, velocities, positions, torques;
    for (int i = 0; i < msg->name.size(); i++)
    {
        string name = msg->name[i];
        dynamixel_info info = joint2dynamixel[name];
        dynamixel_status &status = id2status[info.id];

        //prep the servo, make sure its enabled and in the right mode
        if(status.mode != new_mode && new_mode != UNKOWN)
        {
            if (status.torque_enabled)//you can't seem to change modes while the the servo is enabled
                driver->setTorqueEnabled(info.id, 0);
            status.torque_enabled = false;

            driver->setOperatingMode(info.id, new_mode);//the enum is set up to correspond to the operating modes
            status.mode = new_mode;
        }

        if (!status.torque_enabled)
            driver->setTorqueEnabled(info.id, 1);
        status.torque_enabled = true;

        //prepare data to be sent to the motor
        ids.push_back(info.id);

        if (has_pos)
        {
            double rad_pos = msg->position[i];
            int pos = (int) (rad_pos / 2.0 / M_PI * info.cpr);
            positions.push_back(pos);
        }
        if (has_vel)
        {
            double rad_s_vel = msg->velocity[i];
            int vel = (int) (rad_s_vel / 2.0 / M_PI * 60.0 * info.gear_reduction);
            velocities.push_back(vel);
        }
        if (has_torque)
        {
            //not sure what to do yet. If someone knows the units here, let me know!
        }

        
    }

    //send the commands in batches so it doesn't take as long
    if (has_pos && has_vel)
    {
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
        //do something... later
    }
}

void DynamixelProController::publishJointStates(const ros::TimerEvent& e)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)
    {
        string joint_name = iter->first;
        dynamixel_info info = iter->second;

        int position, velocity;
        if (driver->getPosition(info.id, position))
        {
            double rad_pos = position / ((double) (info.cpr)) * 2 * M_PI;
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
  controller.startListeningForCommands();
  controller.startBroadcastingJointStates();

  ros::spin();//use a single threaded spinner as I'm pretty sure this code isn't thread safe. 
}









