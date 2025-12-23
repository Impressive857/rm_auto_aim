//
// Created by lbw on 25-3-16.
//

#ifndef PRESET_H
#define PRESET_H
#include <visualization_msgs/msg/marker.hpp>

namespace Preset
{
    using Marker = visualization_msgs::msg::Marker;

    struct Preset
    {
        Marker armor;
        Marker center;
        Marker trajectory;
        Marker velocity;
        Marker angle_w;
        Marker num;
    };

    inline Preset getPreset_1(const std::string &frame_id)
    {
        Preset preset;
        preset.armor.header.frame_id = frame_id;
        preset.center.header.frame_id = frame_id;
        preset.trajectory.header.frame_id = frame_id;
        preset.velocity.header.frame_id = frame_id;
        preset.angle_w.header.frame_id = frame_id;
        preset.num.header.frame_id = frame_id;

        preset.armor.ns = "armors";//装甲板
        preset.armor.action = Marker::ADD;
        preset.armor.type = Marker::CUBE;
        preset.armor.scale.x = 0.03;
        preset.armor.scale.z = 0.125;
        preset.armor.color.a = 1.0;
        preset.armor.color.g = 1.0;

        preset.center.ns = "center";
        preset.center.action = Marker::ADD;
        preset.center.type = Marker::SPHERE;
        preset.center.scale.x = preset.center.scale.y = preset.center.scale.z = 0.2;//半径设置为20cm
        preset.center.color.a = 1.0;
        preset.center.color.b = 1.0;

        preset.trajectory.ns = "trajectory";
        preset.trajectory.action = Marker::ADD;
        preset.trajectory.type = Marker::ARROW;
        preset.trajectory.scale.x = 5.0;//线宽设置为2cm
        preset.trajectory.scale.y = 0.04;
        preset.trajectory.scale.z = 0.04;
        preset.trajectory.color.a = 1.0;
        preset.trajectory.color.b = 1.0;

        preset.velocity.ns = "velocity";
        preset.velocity.action = Marker::ADD;
        preset.velocity.type = Marker::ARROW;
        preset.velocity.scale.x = 0.3;/*!需要设置为速度实际值*/
        preset.velocity.scale.y = 0.04;
        preset.velocity.scale.z = 0.04;
        preset.velocity.color.a = 1.0;
        preset.velocity.color.b = 1.0;
        preset.velocity.color.g = 1.0;

        preset.angle_w.ns = "angle_w";
        preset.angle_w.action = Marker::ADD;
        preset.angle_w.type = Marker::ARROW;/*!使用旋转向量表示旋转*/
        preset.angle_w.scale.x = 0.3;/*!需要设置为速度实际值*/
        preset.angle_w.scale.y = 0.04;
        preset.angle_w.scale.z = 0.04;
        preset.angle_w.color.a = 1.0;
        preset.angle_w.color.r = 1.0;
        preset.angle_w.color.g = 1.0;

        preset.num.ns = "detected_num";
        preset.num.action = Marker::ADD;
        preset.num.type = Marker::TEXT_VIEW_FACING;
        preset.num.scale.x = 0.01;
        preset.num.scale.y = 0.01;
        preset.num.scale.z = 0.01;
        preset.num.color.a = 1.0;
        preset.num.color.b = 1.0;
        preset.num.color.g = 1.0;
        preset.num.color.r = 1.0;
        return preset;
    }
}

#endif //PRESET_H
