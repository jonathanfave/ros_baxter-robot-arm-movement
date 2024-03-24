/**
 * \file puppet_hand_node
 * \brief 
 * \author FAVE Jonathan
 * \version 0.1
 * \date 15/03/24
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <baxter_core_msgs/JointCommand.h>

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "puppet_hand_node_03");
    ros::NodeHandle nh_glob;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);    //objet qui est capable d'ecouter le topic tf , capable de donner les positios relative de 2 repères qui appartiennent au meme arbre
    geometry_msgs::TransformStamped transform;

    ros::ServiceClient ik_solver_client = nh_glob.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
    ros::Publisher pub_command = nh_glob.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);

    // Boucle
    ros::Rate rate(10);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();
        try{
            // Donne la position relative depuis notre repere tf fictif créé avec le broadcast avec notre base
            transform = tfBuffer.lookupTransform("base", "frame_03",ros::Time(0)); //("par rapport à", "depuis", instant, ici = maintenant)
        }
        catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
            continue ;
        }

        //ROS_INFO_STREAM("transform:" << transform );

        if(ik_solver_client.exists()){ 
            geometry_msgs::PoseStamped pose_stamped;

            // Recupere la position (x,y,z) et le quaternion (x,y,z,w) actuel de notre bras maitre
            pose_stamped.header.stamp       = ros::Time::now();
            pose_stamped.header.frame_id    = transform.header.frame_id;
            pose_stamped.pose.position.x    = transform.transform.translation.x;
            pose_stamped.pose.position.y    = transform.transform.translation.y;
            pose_stamped.pose.position.z    = transform.transform.translation.z;
            pose_stamped.pose.orientation.x = transform.transform.rotation.x;
            pose_stamped.pose.orientation.y = transform.transform.rotation.y;
            pose_stamped.pose.orientation.z = transform.transform.rotation.z;
            pose_stamped.pose.orientation.w = transform.transform.rotation.w;

            // Le noeud envoie une requete au service
            baxter_core_msgs::SolvePositionIK::Request ik_request;
            baxter_core_msgs::SolvePositionIK::Response ik_response;

            ik_request.pose_stamp.push_back(pose_stamped);
            ik_request.seed_mode = ik_request.SEED_CURRENT; // pos actuelle

            if(ik_solver_client.call(ik_request, ik_response)){
                // si le service trouve une solution à l'équation
                if(ik_response.isValid[0]){
                    ROS_INFO("Traitment done");
                    // publie la commande pour bouger mon  bras esclave en consequence
                    baxter_core_msgs::JointCommand command_joint;
                    command_joint.mode = command_joint.POSITION_MODE;
                    command_joint.names = ik_response.joints[0].name;
                    command_joint.command = ik_response.joints[0].position;

                    pub_command.publish(command_joint);
                } else ROS_ERROR("Traitment failed");
            } else ROS_ERROR("Failled to call service SolvePositionIK");

        }

        rate.sleep();
    }
}
