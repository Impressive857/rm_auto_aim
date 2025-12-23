//
// Created by lbw on 25-5-3.
//

#ifndef REGISTER_NODE_MACRO_H
#define REGISTER_NODE_MACRO_H
#include <factory/load_node_factory.h>
#include <factory/load_static_node_factory.h>
#include <global_node_map.hpp>

#define REGISTER_NODE(Node)\
    LOAD_FACTORY_WITH_COUNT(Node,__COUNTER__,__FILE__)

#define REGISTER_STATIC_NODE(Node)\
    LOAD_STATIC_FACTORY_WITH_COUNT(Node,__COUNTER__,__FILE__)

#endif //REGISTER_NODE_MACRO_H
