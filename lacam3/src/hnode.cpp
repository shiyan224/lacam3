#include "../include/hnode.hpp"

#include <random>

int HNode::COUNT = 0;

HNode::HNode(Config _C, DistTable *D, HNode *_parent, int _g, int _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      corridorPriorities(C.size(), 0),
      movingPriorities(C.size(), 0),
      priorities(C.size(), 0),
      order(C.size(), 0),
      search_tree(std::queue<LNode *>())
{
  ++COUNT;

  search_tree.push(new LNode());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) {
    neighbor.insert(parent);
    parent->neighbor.insert(this);
  }

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (auto i = 0; i < N; ++i) priorities[i] = (float)D->get(i, C[i]) / 10000;
  } else {
    // dynamic priorities, akin to PIBT
    for (auto i = 0; i < N; ++i) {
      if (D->get(i, C[i]) != 0) {
        movingPriorities[i] = parent->movingPriorities[i] + 1;
      } else {
        movingPriorities[i] = parent->movingPriorities[i] - (int)parent->movingPriorities[i];
      }
      if (C[i]->neighbor.size() == 2) { // 在走廊里
        if (D->get(i, C[i]) >= D->get(i, parent->C[i])) { // 逆向进入走廊
          if (parent->corridorPriorities[i])
            corridorPriorities[i] = parent->corridorPriorities[i] - 1;
          else {
            int len = 0;
            Vertex* pre = parent -> C[i];
            Vertex* cur = C[i];
            while (cur -> neighbor.size() == 2) {
              len++;
              for (auto v:cur->neighbor)
                if (v != pre) {
                  pre = cur;
                  cur = v;
                }
            }
          }
        }
        else { // 正向进入走廊
          corridorPriorities[i] = parent->corridorPriorities[i] + 1;
        }
      }
      else corridorPriorities[i] = 0;
      priorities[i] = (float)corridorPriorities[i] + movingPriorities[i];
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int i, int j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

LNode *HNode::get_next_lowlevel_node(std::mt19937 &MT)
{
  if (search_tree.empty()) return nullptr;

  auto L = search_tree.front();
  search_tree.pop();
  if (L->depth < C.size()) {
    auto i = order[L->depth];
    auto cands = C[i]->neighbor;
    cands.push_back(C[i]);
    std::shuffle(cands.begin(), cands.end(), MT);  // randomize
    for (auto u : cands) search_tree.push(new LNode(L, i, u));
  }
  return L;
}

std::ostream &operator<<(std::ostream &os, const HNode *H)
{
  os << "f=" << std::setw(6) << H->f << "\tg=" << std::setw(6) << H->g
     << "\th=" << std::setw(6) << H->h << "\tQ=" << H->C;
  return os;
}

bool CompareHNodePointers::operator()(const HNode *l, const HNode *r) const
{
  const auto N = l->C.size();
  for (auto i = 0; i < N; ++i) {
    if (l->C[i] != r->C[i]) return l->C[i]->id < r->C[i]->id;
  }
  return false;
}
