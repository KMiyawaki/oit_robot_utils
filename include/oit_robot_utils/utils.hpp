#pragma once
#include <string>

template <typename T, typename NodeT>
T declareOrGetParameter(NodeT *node, const std::string &name, const T &default_value)
{
  if (!node->has_parameter(name))
  {
    return node->template declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).template get_value<T>();
}

template <typename T, typename NodeT>
void loadParameter(NodeT *node, const std::string &name, const T &default_value, T &dst)
{
  dst = declareOrGetParameter<T, NodeT>(node, name, default_value);
}

template <typename T, typename NodeT>
void loadParameter(NodeT *node, const std::string &name, T &dst)
{
  dst = declareOrGetParameter<T, NodeT>(node, name, dst);
}
