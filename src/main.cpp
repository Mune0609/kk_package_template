#include "rclcpp/rclcpp.hpp"
#include "kk_driver_msg/msg/key_ctrl.hpp"
#include "kk_driver_msg/msg/pwm_cmd.hpp"
#include "kk_driver_msg/msg/motor_cmd.hpp"
#include "kk_driver_msg/msg/bldc_cmd.hpp"
#include "kk_driver_msg/msg/c610_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

class core_node : public rclcpp::Node{
public:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<kk_driver_msg::msg::KeyCtrl>::SharedPtr sub_key;
    rclcpp::Publisher<kk_driver_msg::msg::MotorCmd>::SharedPtr motor_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::BldcCmd>::SharedPtr bldc_cmd_sub;
    rclcpp::Publisher<kk_driver_msg::msg::C610Cmd>::SharedPtr c610_cmd_sub;

    kk_driver_msg::msg::MotorCmd motor1_cmd_msg;
    kk_driver_msg::msg::MotorCmd motor2_cmd_msg;
    kk_driver_msg::msg::MotorCmd motor3_cmd_msg;
    kk_driver_msg::msg::BldcCmd bldc_cmd_msg;
    kk_driver_msg::msg::C610Cmd c610_cmd_msg;

    core_node() : Node("core_node_node"){

        // Joyコンのサブスクライバー
        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&core_node::joy_callback, this, std::placeholders::_1)
        );
        // キーボードのサブスクライバー (CoRE2用)
        sub_key = this->create_subscription<kk_driver_msg::msg::KeyCtrl>(
            "/keys" , rclcpp::QoS(10),
            std::bind(&core_node::key_callback , this,
            std::placeholders::_1)
        );
        motor_cmd_sub = this->create_publisher<kk_driver_msg::msg::MotorCmd>("mtr/cmd", rclcpp::QoS(10));
        bldc_cmd_sub = this->create_publisher<kk_driver_msg::msg::BldcCmd>("bldc/cmd", rclcpp::QoS(10));
        c610_cmd_sub = this->create_publisher<kk_driver_msg::msg::C610Cmd>("c610/cmd", rclcpp::QoS(10));
    }
    
    bool mode = 0;
    int launch = 0;
    float x_duty;
    float y_duty;
    float duty[4]={0.0f};
    float tt_rot = 0;
    //int X , Y = 0; //計算に使う座標

    float br_left;
    float br_right;

    void Yonrin(float x, float y)       //四輪オムニのpwm値出し
    {
        duty[0] = -x + y;
        duty[1] = -x - y;
        duty[2] =  x - y;
        duty[3] =  x + y;
        printf("a=%f , b=%f ,c=%f ,d=%f\n", duty[0], duty[1], duty[2], duty[3]);

        // auto motor_cmd_msg = kk_driver_msg::msg::MotorCmd();

        //送信するポート数に合わせて配列長を設定
        motor1_cmd_msg.child_id = 0;
        motor1_cmd_msg.port.resize(2);
        motor1_cmd_msg.ctrl.resize(2);
        motor1_cmd_msg.target.resize(2);

        motor2_cmd_msg.child_id = 1;
        motor2_cmd_msg.port.resize(2);
        motor2_cmd_msg.ctrl.resize(2);
        motor2_cmd_msg.target.resize(2);
        
        motor1_cmd_msg.port[0] = 0;
        motor1_cmd_msg.ctrl[0] = 1;
        motor1_cmd_msg.target[0] = static_cast<int32_t>(duty[0] * 0x3FFFFF);

        motor1_cmd_msg.port[1] = 1;
        motor1_cmd_msg.ctrl[1] = 1;
        motor1_cmd_msg.target[1] = static_cast<int32_t>(duty[1] * 0x3FFFFF);

        motor2_cmd_msg.port[0] = 0;
        motor2_cmd_msg.ctrl[0] = 1;
        motor2_cmd_msg.target[0] = static_cast<int32_t>(duty[2] * 0x3FFFFF);

        motor2_cmd_msg.port[1] = 1;
        motor2_cmd_msg.ctrl[1] = 1;
        motor2_cmd_msg.target[1] = static_cast<int32_t>(duty[3] * 0x3FFFFF);

    }

    // Joyコンの処理
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if (msg->buttons.size() < 3) return;

        // auto motor_cmd_msg = kk_driver_msg::msg::MotorCmd();
        // auto bldc_cmd_msg = kk_driver_msg::msg::BldcCmd();
        // auto c610_cmd_msg = kk_driver_msg::msg::C610Cmd();

        if(msg->axes[0] < -0.2 || msg->axes[0] > 0.2 || msg->axes[1] < -0.2 || msg->axes[1] > 0.2){ //4輪オムニ
            x_duty = msg->axes[0];
            y_duty = msg->axes[1];

            Yonrin(x_duty, y_duty);
        }else{
            x_duty = 0;
            y_duty = 0;

            Yonrin(x_duty, y_duty);
        }

        printf("x_duty=%f, y_duty=%f\n", x_duty, y_duty);

        if (msg->buttons[0]){ //ターンテーブル
            if(tt_rot > 0){
                tt_rot =  tt_rot - 1;

                motor3_cmd_msg.child_id = 2;
                motor3_cmd_msg.port.resize(2);
                motor3_cmd_msg.ctrl.resize(2);
                motor3_cmd_msg.target.resize(2);

                motor3_cmd_msg.port[0] = 0;
                motor3_cmd_msg.ctrl[0] = 1;
                motor3_cmd_msg.target[0] = static_cast<int32_t>(tt_rot * 0x3FFFFF);

            }
        } else if (msg->buttons[3]){
            if(tt_rot < 255){
                tt_rot =  tt_rot + 1;

                motor3_cmd_msg.child_id = 2;
                motor3_cmd_msg.port.resize(2);
                motor3_cmd_msg.ctrl.resize(2);
                motor3_cmd_msg.target.resize(2);

                motor3_cmd_msg.port[0] = 0;
                motor3_cmd_msg.ctrl[0] = 1;
                motor3_cmd_msg.target[0] = static_cast<int32_t>(tt_rot * 0x3FFFFF);
            }
        }

        printf("tt_rot=%f\n", tt_rot);

        if (msg->buttons[4]){ //射出モード切替
            br_left = 0x3FFFFF;
            br_right = 0x3FFFFF;
        } else {
            br_left = 0x0FFFFF;
            br_right = 0x0FFFFF;
        }

        bldc_cmd_msg.child_id = 0;
        bldc_cmd_msg.port.resize(2);
        bldc_cmd_msg.spd.resize(2);

        bldc_cmd_msg.port[0] = 0;
        bldc_cmd_msg.spd[0] = br_left;

        bldc_cmd_msg.port[1] = 1;
        bldc_cmd_msg.spd[1] = br_right;

        if(msg->buttons[2]){ //発射
            while(launch > 360){
                launch = launch + 1;

                c610_cmd_msg.child_id = 0;
                c610_cmd_msg.port.resize(1);
                c610_cmd_msg.torque.resize(1);

                // 複数ポート分の設定
                c610_cmd_msg.port[0] = 0;// ポート0のサーボ
                c610_cmd_msg.torque[0] = 0x000F; // ポート0の指令値
                
                printf("launch=%d\n", launch);

            }
            launch = 0;
        }

        printf("br_left=%f, br_right=%f\n", br_left, br_right);
        
       
        motor_cmd_sub->publish(motor1_cmd_msg);
        motor_cmd_sub->publish(motor2_cmd_msg);
        motor_cmd_sub->publish(motor3_cmd_msg);
        bldc_cmd_sub->publish(bldc_cmd_msg);
        c610_cmd_sub->publish(c610_cmd_msg);

    }

    // キー入力の処理
    void key_callback(const kk_driver_msg::msg::KeyCtrl::SharedPtr msg){

        /*クライアントからの変位
        int Xpoti = msg.x;
        //int Ypoti = msg.y;

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
        pub_pwm_cmd.publish(pwm_cmd_msg);


        //マウスの位置と最大値、最小値
        double pulse_calculate( int AmousePoti , int Amax_min){
            int result_pulse = 1500 + 500 * AmousePoti/Amax_min ;
            return result_pulse;
        }*/

    }
    
};


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