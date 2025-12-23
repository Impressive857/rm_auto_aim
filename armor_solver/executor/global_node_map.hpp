//
// Created by lbw on 25-5-4.
//

#ifndef GLOBAL_NODE_MAP_H
#define GLOBAL_NODE_MAP_H
#include <map>
#include <memory>
#include <factory/factory_init_shut.h>

#define NODE_NAME_REF_WITH_COUNTER_ID(Origin, REF, UniqueID)\
namespace global_node{\
    extern std::shared_ptr<Origin> REF;\
    namespace GLOBAL_NODE ## UniqueID{\
        typedef Origin _node;\
        typedef rclcpp::Node _base;\
        typedef std::shared_ptr<_node> _node_SharedPtr;\
        struct Startup ## UniqueID{\
            Startup ## UniqueID(){\
            load_factory::map::global_node_map[# Origin] = \
            [&](std::shared_ptr<rclcpp::Node> node) mutable \
            { global_node::REF = std::static_pointer_cast<_node>(node);  };\
            };\
        };\
    static Startup ## UniqueID g_startup_ ## UniqueID;\
    }\
}

#define NODE_NAME_REF_WITH_COUNTER(Origin, REF, COUNT)\
    NODE_NAME_REF_WITH_COUNTER_ID(Origin, REF, COUNT)

#define NODE_NAME_DEF(Origin, REF)\
   NODE_NAME_REF_WITH_COUNTER(Origin,REF, __COUNTER__);



#endif //GLOBAL_NODE_MAP_H