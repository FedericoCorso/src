#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "eigen3/Eigen/Dense" 
#include <vector>
#include "bits/stdc++.h"

// NODE TO COMPUTE ANGULAR VELOCITIES AND PUBLISH THEM ON CMD_VEL TOPIC

// callback function to retrieve data from bags and fill in a datastructure
void bagCallback(std::vector<double>& aux,const sensor_msgs::JointState::ConstPtr& msg2){ //std::vector<double>& aux argument that I want to implement double* w1, double* w2, double* w3, double* w4,
    
    ROS_INFO("DEBUG: bagCallback funct. called \n");

        for(int i = 0; i < 4; i++){
            aux.push_back(msg2->position[i]);
            ROS_INFO("DEBUG: new elements in the vector:  %lf", msg2->position[i]);
        }
}


int main(int argc, char**argv){
    
    ros::init(argc, argv, "velocities"); //node initialization
    ros::NodeHandle nh; // Node handler
    ros::Rate loop_rate(40); //setting node execution rate
    geometry_msgs::TwistStamped msg1;   
    
    // this will be our d.s. to holds data and make computations.
    std::vector<double> aux {}; // data structure
    std::vector<double>::iterator it1, it2;
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
    
    // subscriber to read from bags //to be modified to use boost bind
    ros::Subscriber read_bag = nh.subscribe<sensor_msgs::JointState>("wheel_states",1000,boost::bind(&bagCallback,boost::ref(aux),_1));
    

    // taking parameters
    double r;
    double l;
    double w;
    double N;
    double T;
    double pi = 2 * acos(0.0);
    double alpha = (2*pi/(N*T)); // resolution of the encoder i radiants
    double f = 50; // sampling time of the encoder
    
    // get parameters from the launch file
    nh.getParam("/r",r);
    nh.getParam("/l",l);
    nh.getParam("/w",w);
    nh.getParam("/N",N);
    nh.getParam("/T",T);
    
    // build the matrix of coefficients
    Eigen::MatrixXd A(4,3);
    
    A(0,0) = -l-w; A(0,1) = 1; A(0,2) = -1;
    A(1,0) =  l+w; A(1,1) = 1; A(1,2) =  1;
    A(2,0) =  l+w; A(2,1) = 1; A(2,2) = -1;
    A(3,0) = -l-w; A(3,1) = 1; A(3,2) =  1;
    
    // Pseudo inverse matrix
    Eigen::MatrixXd A_1 = A.transpose()*A;
    Eigen::MatrixXd A_1_inv = A_1.inverse();

    // vector of known terms
    Eigen::MatrixXd b(4,1);

    // unknowns vector
    Eigen::MatrixXd x(3,1); 

    // build the vector of known parameters: angular velocities need to read from bag
    while(ros::ok()){

        // check if new messages have been published on wheel_states
        ros::spinOnce();
        
        // check if the bag has been called

        if(aux.empty()==false){

            ROS_INFO("DEBUG: Bag is running so I can make computations");

            //check the size of aux d.s.
            if(aux.size()==4){

                ROS_INFO("Vector contains only 4 elements, can't do computations since t = 0");
               
                for(int i = 0; i < aux.size(); i++){

                    ROS_INFO("DEBUG: aux[%d]: %lf",i, aux[i]);
                
                }
            
            }else{
                
                ROS_INFO("aux size > 4");

                for(int i = 0; i < aux.size(); i++){
                    ROS_INFO("DEBUG: aux[%d]: %lf",i,aux[i]);
                }
                

                 //b(i,0) = (auk[k+1]-aux[k])*alpha*f;
                
                b(0,0) = (aux[4]-aux[0])*alpha*f;
                b(1,0) = (aux[5]-aux[1])*alpha*f;
                b(2,0) = (aux[6]-aux[2])*alpha*f;
                b(3,0) = (aux[7]-aux[3])*alpha*f;

                for(int i = 0; i < 4; i++){
                    ROS_INFO("DEBUG: b[%d]: %lf",i, b(i,0));
                }

                ROS_INFO("DEBUG: computing OLS solution from ticks and converting into rad/s \n");
            
                x = A_1_inv*A.transpose()*b; // inefficient line

                ROS_INFO("DEBUG: building the message \n");
                
                msg1.twist.linear.x = x(0,0);
                msg1.twist.linear.y = x(1,0);
                msg1.twist.angular.z = x(2,0);

                vel_pub.publish(msg1);

                // update the d.s.
               

                it1 = aux.begin();
                it2 = aux.end();
                
                it2--;
                it2--;
                it2--;
                it2--;

                aux.erase(it1,it2);

                
                

            }

        }else{

            ROS_INFO("DEBUG: Bag has not been called, no computation needed");
        }

        loop_rate.sleep();
    }

    return 0;
}