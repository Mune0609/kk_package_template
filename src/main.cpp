#include "rclcpp/rclcpp.hpp"
#include "kk_driver_msg/msg/key_ctrl.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

class kk_template_node : public rclcpp::Node{
public:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<kk_driver_msg::msg::KeyCtrl>::SharedPtr sub_key;

    kk_template_node() : Node("kk_template_node_node"){

        // Joyコンのサブスクライバー
        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&kk_template_node::joy_callback, this, std::placeholders::_1)
        );
        // キーボードのサブスクライバー (CoRE2用)
        sub_key = this->create_subscription<kk_driver_msg::msg::KeyCtrl>(
            "/keys" , rclcpp::QoS(10),
            std::bind(&kk_template_node::key_callback , this,
            std::placeholders::_1)
        );
        
    }

    // Joyコンの処理
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    }

    // キー入力の処理
    void key_callback(const kk_driver_msg::msg::KeyCtrl::SharedPtr msg){
    }

};



int main(int argc, char *argv[]) {
    // ROS 2クライアントライブラリの初期化
    rclcpp::init(argc, argv);
    // ノードの作成と実行
    auto node = std::make_shared<kk_template_node>();
    // スピン処理
    rclcpp::spin(node);
    // ROS 2クライアントライブラリの終了処理
    rclcpp::shutdown();
    return 0;
}