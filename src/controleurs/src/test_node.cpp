#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <string>   



class InitialTest : public rclcpp::Node {
public:
    InitialTest() : Node("initial_test_node") {
        RCLCPP_INFO(this->get_logger(), "=== DÉBUT TEST SIMPLE ==="); // dès que l'objet est créé ce messsage s'affiche dans le terminal
        
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); //initialisation du publisher sur le topic /joint_states
        
        joint_names_ = {"rail_joint", "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"}; // noms des joints (rail + 7 joints du bras)
        
        // Position qu'on veut attribuer aux joints
        rail_position_ = 0.0;
        arm_joints_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1),  // 50 Hz normalement mais pour ce test on fera 1 Hz
            std::bind(&InitialTest::publishJointStates, this) // liaison avec la fonction de publication
        );
        
        RCLCPP_INFO(this->get_logger(), "=== TEST SIMPLE OK - Publishing at 1Hz ==="); // message de confirmation dans le terminal
    }

private:
    void publishJointStates() {
        sensor_msgs::msg::JointState msg; // création du message JointState
        msg.header.stamp = this->now(); // timestamp actuel
        msg.name = joint_names_; // attribution des noms des joints
        msg.position.reserve(8); // réserver de l'espace pour 8 positions (1 rail + 7 joints)
        msg.position.push_back(rail_position_); // position du rail mise en premier
        msg.position.insert(msg.position.end(), arm_joints_.begin(), arm_joints_.end()); // ajout des positions des joints du bras en suite
        pub_->publish(msg);
    }
    
    double rail_position_;
    std::vector<double> arm_joints_;
    std::vector<std::string> joint_names_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialTest>());
    rclcpp::shutdown();
    return 0;
}