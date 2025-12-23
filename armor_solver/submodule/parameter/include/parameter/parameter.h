//
// Created by lbw on 25-5-4.
//

#ifndef PARAMETER_H
#define PARAMETER_H
#include <rclcpp/rclcpp.hpp>
namespace ckyf
{
    namespace parameter
    {
        class ArmorSolverParam : public rclcpp::Node
        {
        public:
            explicit ArmorSolverParam(const rclcpp::NodeOptions& options);

        private:
            void declare_params();
        };
    }
}

#include <global_node_map.hpp>
NODE_NAME_DEF(ckyf::parameter::ArmorSolverParam, Parameter)
#endif //PARAMETER_H
