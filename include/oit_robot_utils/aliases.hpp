#ifndef ___ALIASES_HPP___
#define ___ALIASES_HPP___

#include <deque>
#include <memory>
#include <set>
#include <vector>

template <typename T>
struct SharedPtrAliases
{
  using Ptr = std::shared_ptr<T>;
  using ConstPtr = std::shared_ptr<const T>;
};

#define DECLARE_SHARED_PTR_ALIASES(T)         \
  using SharedPtrTypes = SharedPtrAliases<T>; \
  using Ptr = typename SharedPtrTypes::Ptr;   \
  using ConstPtr = typename SharedPtrTypes::ConstPtr;

template <typename T>
struct ContainerAliases : public SharedPtrAliases<T>
{
  using Ptr = typename SharedPtrAliases<T>::Ptr;
  using ConstPtr = typename SharedPtrAliases<T>::ConstPtr;

  using Vector = std::vector<Ptr>;
  using ConstVector = std::vector<ConstPtr>;
  using Deque = std::deque<Ptr>;
  using ConstDeque = std::deque<ConstPtr>;
  using Set = std::set<Ptr>;
  using ConstSet = std::set<ConstPtr>;
};

#define DECLARE_CONTAINER_ALIASES(T)                        \
  using ContainerTypes = ContainerAliases<T>;               \
  using Ptr = typename ContainerTypes::Ptr;                 \
  using ConstPtr = typename ContainerTypes::ConstPtr;       \
  using Vector = typename ContainerTypes::Vector;           \
  using ConstVector = typename ContainerTypes::ConstVector; \
  using Deque = typename ContainerTypes::Deque;             \
  using ConstDeque = typename ContainerTypes::ConstDeque;   \
  using Set = typename ContainerTypes::Set;                 \
  using ConstSet = typename ContainerTypes::ConstSet;

#endif
