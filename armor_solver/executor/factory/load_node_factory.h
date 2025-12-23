//
// Created by lbw on 25-5-4.
//

#ifndef LOAD_NODE_FACTORY_H
#define LOAD_NODE_FACTORY_H
#include <rclcpp/rclcpp.hpp>
#include <rm_utils/logger/log.hpp>
#include <factory/factory_init_shut.h>

namespace load_factory
{
    namespace imlp
    {
        template <class _Node>
        void registerPlugin(const std::string& class_name)
        {
            if (map::factory_map[class_name] == nullptr)
            {
                FYT_INFO("load_factory", "Add Node Factory: {}", class_name);
                map::factory_map[class_name] = []
                {
                    return std::make_shared<_Node>(rclcpp::NodeOptions());
                };
            }
        }
    }
}

#define LOAD_FACTORY(NodeType, UniqueID, CallPath)\
namespace {\
    struct Proxy  ## UniqueID{\
    typedef NodeType _node;\
    Proxy ## UniqueID(){     \
    load_factory::init::init();\
    load_factory::imlp::registerPlugin<_node>(# NodeType);\
    load_factory::fs::record_call_path(CallPath, BUILD_DIR, INDEX_NAME);\
    }\
    };\
    static Proxy ## UniqueID g_register_node ## UniqueID;\
}

#define LOAD_FACTORY_WITH_COUNT(Node,Unique,CallPath)\
LOAD_FACTORY(Node,Unique,CallPath)

#endif //LOAD_NODE_FACTORY_H
