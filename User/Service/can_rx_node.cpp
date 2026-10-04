#include "can_rx_node.hpp"

#include "can_bus.hpp"

#include <new>


uint8_t CanRxNode::node_num = 0;

CanRxNode::CanRxNode(const Config &cfg) :
  _can_id(cfg.can_id),
  _id_type(cfg.id_type),
  _callback(cfg.callback),
  _context(cfg.context),
  next(nullptr)
{
}

CanRxNode *regist(const CanRxNode::Config &cfg)
{
  if (sys_event == nullptr || CanBus::frozen || sys_flag_running() || cfg.bus == nullptr)
  {
    return nullptr; // 事件组未就绪，或注册表已冻结
  }
  if (cfg.callback == nullptr || cfg.bus->_can == nullptr || cfg.bus->_can->_hfdcan == nullptr || (cfg.id_type != FDCAN_STANDARD_ID && cfg.id_type != FDCAN_EXTENDED_ID) || cfg.can_id > (cfg.id_type == FDCAN_STANDARD_ID ? 0x7FFU : 0x1FFFFFFFU))
  {
    sys_init_error(Status::BAD_ARG);
    return nullptr;
  }
  for (CanRxNode *node = cfg.bus->rx_head; node != nullptr; node = node->next)
  {
    if (node->_can_id == cfg.can_id && node->_id_type == cfg.id_type)
    {
      sys_init_error(Status::BUSY);
      return nullptr;
    }
  }
  if (CanRxNode::node_num >= 50U)
  {
    sys_init_error(Status::FULL);
    return nullptr;
  }
  CanRxNode *node = new (std::nothrow) CanRxNode(cfg);
  if (node == nullptr)
  {
    sys_init_error(Status::FULL);
    return nullptr;
  }
  node->next       = cfg.bus->rx_head;
  cfg.bus->rx_head = node;
  CanRxNode::node_num++;
  return node;
}

Status unregist(CanBus &bus, CanRxNode *node)
{
  if (CanBus::frozen || sys_flag_running())
  {
    return Status::NOT_SUPPORTED;
  }
  // 不解引用未知指针；只有确认归属后才删除。
  for (CanRxNode **item = &bus.rx_head; *item != nullptr; item = &(*item)->next)
  {
    if (*item == node)
    {
      *item = node->next;
      delete node;
      --CanRxNode::node_num;
      return Status::OK;
    }
  }
  return Status::BAD_ARG;
}
