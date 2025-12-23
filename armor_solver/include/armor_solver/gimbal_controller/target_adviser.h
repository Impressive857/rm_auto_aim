//
// Created by lbw on 25-5-2.
//

#ifndef TARGET_ADVISER_H
#define TARGET_ADVISER_H

#include <rclcpp/rclcpp.hpp>

#include <armor_solver/armor_solver_common.h>

namespace ckyf
{
    namespace auto_aim
    {
        class TargetAdviser
        {
        public:
            enum class AdviceState
            {
                MinDistance, /*!使用距离图像中心最近的装甲板**/
                Extern, /*!使用外部目标建议**/
                Lock, /*!加锁，不会切换目标**/
            };


            explicit TargetAdviser(TargetAdviser&) = delete;
            ~TargetAdviser();
            static TargetAdviser* getInstance();
            void init();
            void lock();
            void unlock();

            void putExternAdvice(const TargetAdvice& advice);
            void putDetectArmorGroup(const ArmorGroup& armor_group);
            std::string getShootTarget();

            AdviceState advice_state;

            static std::string find_min_armor(const ArmorGroup& armor_groups);

        private:
            explicit TargetAdviser();
            static TargetAdviser* instance_;

            void updateShootTarget();
            TargetAdvice extern_target_advice_;
            ShootTarget shoot_target_;
            ArmorGroup armor_group_;
            std::mutex armor_group_mutex_;
            //lock
            std::atomic<bool> target_lock_ = false;
            //parameters
            double drop_time_;

            //tools
            bool examine_timestamp(rclcpp::Time timestamp);
        };

    }
}


#endif //TARGET_ADVISER_H
