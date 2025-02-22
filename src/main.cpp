#include "rclcpp/rclcpp.hpp"
#include "kk_driver_msg/msg/key_ctrl.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"
// サーボ関連
#include "board_msgs/msg/servo_command.hpp"
#include "board_msgs/msg/servo_responce.hpp"
// モータ関連
#include "board_client/motor_client.hpp"
#include "board_client/motor_setting.hpp"
#include "board_msgs/msg/motor_simple.hpp"
#include "board_msgs/msg/axes_confirm.hpp"

class core_node : public rclcpp::Node{
public:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<kk_driver_msg::msg::KeyCtrl>::SharedPtr sub_key;
    int mode = 0;
    int launch = 0;

    core_node() : Node("core_node_node"){

        // Joyコンのサブスクライバー
        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&core::joy_callback, this, std::placeholders::_1)
        );
        // キーボードのサブスクライバー (CoRE2用)
        sub_key = this->create_subscription<kk_driver_msg::msg::KeyCtrl>(
            "/keys" , rclcpp::QoS(10),
            std::bind(&core::key_callback , this,
            std::placeholders::_1)
        );
    }

    if (!test_count){//初期値設定
        board_msgs::msg::ServoCommand msg_port0;
        msg_port0.port = 0;
        msg_port0.pos = tt; //ターンテーブル
        msg_port0.speed = 0.2;
        pwm0_pub->publish(msg_port0);
    }

    if(!joy_init){
        return;
    }

    void fly(int mode){
        if (mode == 0){
            br_left = 1.0;
            br_right = 1.0;
        } else {
            br_left = 1.5;
            br_right = 1.5;
        }

        board_msgs::msg::ServoCommand msg_port000;
        msg_port000.port = 0;
        msg_port000.pos = br_left;
        pwm0_pub->publish(msg_port000);
        // 
        board_msgs::msg::ServoCommand msg_port001;
        msg_port001.port = 1;
        msg_port001.pos = br_right;
        pwm0_pub->publish(msg_port001);
    }

    void Yonrin(float x, float y)       //四輪オムニのpwm値出し
    {
        duty[0] = -x/sqrtf(2) + y/sqrtf(2);
        duty[1] = -x/sqrtf(2) - y/sqrtf(2);
        duty[2] =  x/sqrtf(2) - y/sqrtf(2);
        duty[3] =  x/sqrtf(2) + y/sqrtf(2);
        pc.printf("a=%f , b=%f ,c=%f ,d=%f\n",duty[0],duty[1],duty[2],duty[3]);
            
        board_msgs::msg::MotorSimple msg_port00;
        //msg_port00.rev = false;
        msg_port00.port = 0;
        msg_port00.duty = duty[0];
        cmd0_pub->publish(msg_port00);
        board_msgs::msg::MotorSimple msg_port01;
        //msg_port01.rev = false;
        msg_port01.port = 1;
        msg_port01.duty = duty[1];
        cmd0_pub->publish(msg_port01);
        board_msgs::msg::MotorSimple msg_port02;
        //msg_port02.rev = false;
        msg_port02.port = 0;
        msg_port02.duty = duty[2];
        cmd1_pub->publish(msg_port02);
        board_msgs::msg::MotorSimple msg_port03;
        //msg_port03.rev = false;
        msg_port03.port = 1;
        msg_port03.duty = duty[3];
        cmd1_pub->publish(msg_port03);
    }

    // Joyコンの処理
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

        pad = msg;
        mode = 0;

        if(pad.axes[0] < -0.2 || pad.axes[0] > 0.2 || pad.axes[1] < -0.2 || pad.axes[1] > 0.2){ //4輪オムニ
            float x_duty = ((pad.axes[0] - 128.0f) / 127.0f) * 0.7f;
            float y_duty = -((pad.axes[1] - 128.0f) / 127.0f) * 0.7f;
//                  pc.printf("x_duty=%f y_duty=%f\",x_duty,y_duty);
            Yonrin(x_duty, y_duty);
        }

        if (pad.buton[0]){ //ターンテーブル
            if(tt_rot > 0){
                tt_rot =  tt_rot - 1;
                board_msgs::msg::ServoCommand msg_port0;
                msg_port0.port = 0;
                msg_port0.pos = tt_rot;
                msg_port0.duty = 0.2;
                pwm0_pub->publish(msg_port0);
            }
        } else if (pad.buton[3]){
            if(tt_rot < 255){
                tt_rot =  tt_rot + 1;
                board_msgs::msg::ServoCommand msg_port0;
                msg_port0.port = 0;
                msg_port0.pos = tt_rot;
                msg_port0.duty = -0.2;
                pwm0_pub->publish(msg_port0);
            }
        }

        if (pad.axes[5] > 0.2){ //射出モード切替
            mode = 1;
        } else if (pad.axes[5] <= 0.2){
            mode = 0;
        }

        //ブラシレス
        board_msgs::msg::ServoCommand msg_port000;
        msg_port000.port = 0;
        msg_port000.pos = br_left;
        pwm0_pub->publish(msg_port000);
        board_msgs::msg::ServoCommand msg_port001;
        msg_port001.port = 1;
        msg_port001.pos = br_right;
        pwm0_pub->publish(msg_port001);

        if(pad.button[10]){ //発射
            while(launch > 360){
                board_msgs::msg::ServoCommand msg_port001;
                msg_port001.port = 1;
                msg_port001.pos = launch;
                pwm0_pub->publish(msg_port001);
            }
            launch = 0;
        }

        /*int X , Y = 0; //計算に使う座標

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

    }

    // キー入力の処理
    void key_callback(const kk_driver_msg::msg::KeyCtrl::SharedPtr msg){
    }*/

    }
}

int main(int argc, char *argv[]) {
    // ROS 2クライアントライブラリの初期化
    rclcpp::init(argc, argv);
    // ノードの作成と実行
    auto node = std::make_shared<core_node>();
    // スピン処理
    rclcpp::spin(node);
    // ROS 2クライアントライブラリの終了処理
    rclcpp::shutdown();
    return 0;
}