//
// Created by lbw on 25-5-4.
//

#ifndef LOAD_STATIC_NODE_FACTORY_H
#define LOAD_STATIC_NODE_FACTORY_H
#include <rclcpp/rclcpp.hpp>
#include <rm_utils/logger/log.hpp>
#include <factory/factory_init_shut.h>

namespace load_factory
{
    namespace imlp
    {
        template <class _StaticNode>
        void registerStaticPlugin(const std::string& class_name)
        {
            if (map::factory_map[class_name] == nullptr)
            {
                FYT_INFO("load_factory","Add Static Node Factory: {}",class_name);
                map::factory_map[class_name] = []
                {
                    return _StaticNode::getInstance();
                };
            }

        }
    }
}

#define LOAD_STATIC_FACTORY(NodeType, UniqueID, CallPath)\
    std::shared_ptr<NodeType> NodeType::instance_ = nullptr;\
    namespace {\
    struct Proxy ## UniqueID{\
    typedef NodeType _node;\
    Proxy ## UniqueID(){\
    load_factory::init::init();\
    load_factory::imlp::registerStaticPlugin<_node>(# NodeType);\
    load_factory::fs::record_call_path(CallPath,BUILD_DIR,INDEX_NAME);\
    };\
    };\
    static Proxy ## UniqueID g_register_node ## UniqueID;\
}

#define LOAD_STATIC_FACTORY_WITH_COUNT(Node,UniqueID,CallPath)\
    LOAD_STATIC_FACTORY(Node,UniqueID,CallPath)


#endif //LOAD_STATIC_NODE_FACTORY_H
