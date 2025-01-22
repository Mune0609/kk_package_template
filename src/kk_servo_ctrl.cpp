#include "rclcpp/rclcpp.hpp"
#include "kk_driver_msg/msg/key_ctrl.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"

class kk_servo_ctrl : public rclcpp::Node{
public:
    rclcpp::Subscription<kk_driver_msg::msg::KeyCtrl>::SharedPtr sub_msg;
    rclcpp::Publisher<kk_driver_msg::msg::PwmCmd>::SharedPtr pub_pwm_cmd;
    

    kk_servo_ctrl() : Node("kk_servo_ctrl_node"){

        sub_msg = this->create_subscription<kk_driver_msg::msg::KeyCtrl>(
            "/keys" , rclcpp::QoS(10),
            std::bind(&kk_servo_ctrl::call_back , this,
            std::placeholders::_1)
        );
        pub_pwm_cmd = this->create_publisher<kk_driver_msg::msg::PwmCmd>("/pwm_cmd", rclcpp::QoS(10));
    }
    int X , Y = 0; //計算に使う座標

    void call_back(const kk_driver_msg::msg::KeyCtrl::SharedPtr msg){
        //クライアントからの変位
        int Xpoti = msg->x;
        //int Ypoti = msg->y;

        int max_min = 2000;//最大値、最小値を設定
        if(Xpoti > max_min || Xpoti < max_min*-1){
            if( Xpoti > max_min){
                Xpoti = max_min;
            }else if(Xpoti < max_min*-1){
                Xpoti = max_min*-1;
            }
        }
        X = Xpoti;
        printf("X : %d , Y : %d\n" , X , Y);

        auto pwm_cmd_msg = kk_driver_msg::msg::PwmCmd();
        pwm_cmd_msg.child_id = 0;
        pwm_cmd_msg.port = {0};
        pwm_cmd_msg.pos.resize(1); 
        pwm_cmd_msg.pos[0]= pulse_calculate( X , max_min);
        pwm_cmd_msg.spd= {0xdf};
        pub_pwm_cmd->publish(pwm_cmd_msg);
    }

    


    //マウスの位置と最大値、最小値
    double pulse_calculate( int AmousePoti , int Amax_min){
        int result_pulse = 1500 + 500 * AmousePoti/Amax_min ;
        return result_pulse;
    }
};



int main(int argc, char *argv[]) {
    // ROS 2クライアントライブラリの初期化
    rclcpp::init(argc, argv);
    // ノードの作成と実行
    auto node = std::make_shared<kk_servo_ctrl>();
    // スピン処理
    rclcpp::spin(node);
    // ROS 2クライアントライブラリの終了処理
    rclcpp::shutdown();
    return 0;
}