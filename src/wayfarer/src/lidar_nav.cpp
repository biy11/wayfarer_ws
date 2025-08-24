#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class LidarNav : public rclcpp::Node{
public:
    LidarNav() 
    : Node("lidarnav"), left_average(0.0), right_average(0.0), front_average(0.0) {

        //////////////// PUBLISHERS /////////////////
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",10
        );

        /////////////// SUBSCRIPTIONS ///////////////
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarNav::lidar_callback, this, std::placeholders::_1)
        );
    }

private:

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::vector<float> ranges = msg->ranges;
        // RCLCPP_INFO(this->get_logger(), "Range Size: %d",  ranges.size() );


        this->left_average = get_average_distance(msg, -15, 15);    // front
        this->front_average  = get_average_distance(msg, 75, 105);     // left
        this->right_average = get_average_distance(msg, 155, 185);      // right
        // this->front_average = get_average_distance(msg, -15, 15);
        // this->right_average = get_average_distance(msg, -105, -75);
        // this->left_average = get_average_distance(msg, 75, 105);



        RCLCPP_INFO(this->get_logger(), "Left Average: %.2f",  this->left_average );
        // RCLCPP_INFO(this->get_logger(), "front Average: %.2f",  this->front_average );
        RCLCPP_INFO(this->get_logger(), "right Average: %.2f",  this->right_average );
        RCLCPP_INFO(this->get_logger(), "----------------------------------------" );

        // RCLCPP_INFO(this->get_logger(), "Angle Increment: %.2f", msg->angle_increment);
        // RCLCPP_INFO(this->get_logger(), "Angle Min: %.2f, Max: %.2f", msg->angle_min, msg->angle_max);

        

        navigate();

    }

    float sum(std::vector<float> &data){
        float sum = 0;
        for (const auto &value: data){
            sum+= value;
        }
        return sum;
    }

    float get_average_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg, int angle_start_deg, int angle_end_deg){
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;
        std::vector<float> ranges = msg->ranges;

        // Conversion from dgrees to radiants
        float start_rad = (angle_start_deg * M_PI) / 180;
        float end_rad = (angle_end_deg * M_PI) /180;

        // Clamp indicies within bounds
        int start_index = std::max(0, static_cast<int>(std::floor((start_rad - angle_min) / angle_increment)));
        int end_index = std::min(static_cast<int>(ranges.size()), static_cast<int>(std::floor((end_rad - angle_min) / angle_increment)));

        std::vector<float> valid_values;
        for(int i = start_index; i < end_index; i++){
            if(ranges[i] != INFINITY){
                valid_values.push_back(ranges[i]);
            }
        }
        if (!valid_values.empty()){
            return sum(valid_values) / valid_values.size();
        }
        else{
            return NAN;
        }

    }

    void rover_move(float linear, float angular){
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear;
        msg.angular.z = angular;
        cmd_vel_pub_->publish(msg);
    }

    void navigate(){
        // if(front_average < 0.5){
        //     RCLCPP_INFO(this->get_logger(), "Obstacle detected ahead");
        //     rover_move(0.0, 0.75);
        // }else if(right_average < 0.5){
        //     RCLCPP_INFO(this->get_logger(), "Turning left");
        //     rover_move(0.0, -0.75);
        // }else{
        //     RCLCPP_INFO(this->get_logger(), "Moving forwards");
        //     rover_move(1.5, 0.0);
        // }

        if(front_average < 0.5){
            if(std::abs(right_average - left_average) < 0.4){
                RCLCPP_INFO(this->get_logger(), "Obstacle detected - Turning left (default)");
                rover_move(0.0, 0.75);
            }else if(right_average > left_average){
                RCLCPP_INFO(this->get_logger(), "Turning left");
                rover_move(0.0, 0.75);
            }else{
                RCLCPP_INFO(this->get_logger(), "Turning right");
                rover_move(0.0, -0.75);
            }
        }else{
            RCLCPP_INFO(this->get_logger(), "Moving forward");
            rover_move(1.5, 0.0);
        }

        
        
    }

    float left_average;
    float right_average;
    float front_average;  
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNav>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;


}