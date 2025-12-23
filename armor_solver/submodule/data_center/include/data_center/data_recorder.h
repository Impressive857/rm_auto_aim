//
// Created by lbw on 25-5-3.
//

#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H
#include <DataCenterCommon>
#include <mutex>
#include <set>
#include <rclcpp/node.hpp>
#include <rm_interfaces/msg/chassis.hpp>
#include <rm_interfaces/msg/operator.hpp>
#include <rm_interfaces/srv/enemy_strategist.hpp>
#include <clock/clock.h>

namespace ckyf
{
    namespace data_center
    {
        using Chassis = rm_interfaces::msg::Chassis;
        using Operator = rm_interfaces::msg::Operator;
        using EnemyStrategist = rm_interfaces::srv::EnemyStrategist;

        template<class T>
        struct State
        {
            std_msgs::msg::Header header;
            T data;
            bool valid{};
        };

        using ChassisState = State<Chassis>;
        using OperatorState = State<Operator>;
        class DataRecorder : public rclcpp::Node
        {
        public:
            ~DataRecorder()=default;
            DataRecorder(rclcpp::NodeOptions);
            DataRecorder(DataRecorder const&) = delete;
            DataRecorder& operator=(DataRecorder const&) = delete;

            //注册观测的合法id
            bool registerRobotID(const std::string& robot_id);

            //观测状态更新
            void mainDetectEnemy(const EnemyPosition& enemy);
            void sensorDetectEnemy(const EnemyPosition& enemy);

            //更新底盘状态
            void updateChassis(const Chassis& chassis);
            bool getChassis(Chassis& chassis) const;

            //更新操作情况
            void updateOperator(const Operator& op);
            bool getOperator(Operator& op) const;

            //查询当前的目标建议
            rclcpp::Client<EnemyStrategist>::FutureAndRequestId ask_for_strategy();
            bool serviceOn;

            //tools
            bool is_id_valid(const std::string& robot_id);

            EnemyPositions::SharedPtr getEnemies(){return enemy_positions_;}

        private:

            template<class T>
            void checkValid(T& state);

            std::shared_ptr<EnemyPositions> enemy_positions_{nullptr};
            std::set<std::string> valid_id_;

            rclcpp::Client<EnemyStrategist>::SharedPtr strategist;
            rclcpp::TimerBase::SharedPtr update_timer_;
            void update_timer_callback();

            //底盘数据
            ChassisState chassis_state_;
            //操作手数据
            OperatorState operator_state_;
        };

        template <class T>
        void DataRecorder::checkValid(T& state)
        {
            rclcpp::Time time = global_node::Clock->time();
            auto duration = time - chassis_state_.header.stamp;
            if (duration.seconds() > 0.1)
            {
                state.valid = false;
            }
        }
    }
}



#include <global_node_map.hpp>
NODE_NAME_DEF(ckyf::data_center::DataRecorder,DataRecorder)

#endif //DATA_RECORDER_H
