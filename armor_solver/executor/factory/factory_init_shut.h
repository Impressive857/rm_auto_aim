//
// Created by lbw on 25-5-4.
//

#ifndef FACTORY_INIT_SHUT_H
#define FACTORY_INIT_SHUT_H
#include <fstream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rm_utils/logger/log.hpp>

namespace load_factory
{
    namespace map
    {
        extern std::map<std::string, std::shared_ptr<rclcpp::Node>> node_map;
        extern std::map<std::string, std::function<rclcpp::Node::SharedPtr()>> factory_map;
        extern std::map<std::string, std::function<void(std::shared_ptr<rclcpp::Node>)>> global_node_map;
    }

    namespace init
    {
        static void* isInit = nullptr;

        inline void initLogger()
        {
            FYT_REGISTER_LOGGER("load_factory", "~/fyt2024-log", INFO);
        }

        inline void init()
        {
            if (isInit == nullptr)
            {
                initLogger();
                isInit = malloc(1);
            }
        }

        [[maybe_unused]] static bool spinning = false;

        static void* instantiation_done = nullptr;

        inline void instantiation()
        {
            if (instantiation_done != nullptr)
            {
                FYT_ERROR("load_factory", "double instantiation");
                return;
            }
            instantiation_done = malloc(1);
            for (const auto& [name,factory] : map::factory_map)
            {
                if (map::node_map[name] == nullptr)
                {
                    map::node_map[name] = factory();
                    FYT_INFO("load_factory", "Instantiated Node {}", name);
                }
                else
                {
                    FYT_WARN("load_factory", "Instantiated Node {} already exists", name);
                }
            }
        }

        static void* refresh_done = nullptr;

        inline void refresh_reference()
        {
            if (refresh_done != nullptr)
            {
                FYT_WARN("load_factory", "double refresh");
                return;
            }
            refresh_done = malloc(1);
            for (const auto& [name,node_SharedPtr] : map::node_map)
            {
                if (map::global_node_map[name] != nullptr)
                {
                    map::global_node_map[name](node_SharedPtr);
                    FYT_INFO("load_factory", "Refreshed Node {}", name);
                }
            }
        }
    }

    namespace fs
    {
        static void* write_header = nullptr;

        inline void record_call_path(const std::string& call_path, const std::string& build_dir,
                                     const std::string& index_name)
        {
            if (write_header == nullptr)
            {
                std::fstream fs(build_dir + "/" + index_name, std::ios::out);
                fs << "#pragma once" << std::endl;
                write_header = malloc(1);
                fs.close();
            }
            std::fstream fs(build_dir + "/" + index_name, std::ios::out | std::ios::app);
            fs << "#include <" << call_path << ">" << std::endl;
            fs.close();
        }
    }

    namespace shutdown
    {
        inline void shutdown()
        {
            if (init::isInit != nullptr)
            {
                free(init::isInit);
                init::isInit = nullptr;
            }
            if (fs::write_header != nullptr)
            {
                free(fs::write_header);
                fs::write_header = nullptr;
            }
            if (init::instantiation_done != nullptr)
            {
                free(init::instantiation_done);
                init::instantiation_done = nullptr;
            }
            if (init::refresh_done != nullptr)
            {
                free(init::refresh_done);
                init::refresh_done = nullptr;
            }
        }
    }
}


#endif //FACTORY_INIT_SHUT_H
