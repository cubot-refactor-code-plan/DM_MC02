#include "can_rx_node.hpp"

#include <new>

uint8_t CanRxNode::node_num = 0;
bool CanRxNode::frozen = false;

CanRxNode::CanRxNode(uint32_t can_id, Callback callback, void *context, uint32_t id_type) :
  _can_id(can_id), _id_type(id_type), _callback(callback), _context(context), next(nullptr)
{
}

bool CanRxNode::is_frozen(void)
{
  if (sysEvent != nullptr && (osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0U)
  {
    frozen = true;
  }
  return frozen;
}

CanRxNode *regist(BspCan &can_item, uint32_t can_id, CanRxNode::Callback callback, void *context, uint32_t id_type)
{
  if (sysEvent == nullptr || CanRxNode::is_frozen())
  {
    return nullptr;
  }
  if (callback == nullptr || can_item._hfdcan == nullptr ||
      (id_type != FDCAN_STANDARD_ID && id_type != FDCAN_EXTENDED_ID) ||
      can_id > (id_type == FDCAN_STANDARD_ID ? 0x7FFU : 0x1FFFFFFFU))
  {
    SysInitError(Status::BAD_ARG);
    return nullptr;
  }
  for (CanRxNode *node = can_item.rx_head; node != nullptr; node = node->next)
  {
    if (node->_can_id == can_id && node->_id_type == id_type)
    {
      SysInitError(Status::BUSY);
      return nullptr;
    }
  }
  if (CanRxNode::node_num >= 50U)
  {
    SysInitError(Status::FULL);
    return nullptr;
  }
  CanRxNode *node = new (std::nothrow) CanRxNode(can_id, callback, context, id_type);
  if (node == nullptr)
  {
    SysInitError(Status::FULL);
    return nullptr;
  }
  node->next = can_item.rx_head;
  can_item.rx_head = node;
  ++CanRxNode::node_num;
  return node;
}

Status unregist(BspCan &can_item, CanRxNode *node)
{
  if (CanRxNode::is_frozen())
  {
    return Status::NOT_SUPPORTED;
  }
  // 不解引用未知指针；只有确认归属后才删除。
  for (CanRxNode **item = &can_item.rx_head; *item != nullptr; item = &(*item)->next)
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
