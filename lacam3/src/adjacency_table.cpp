//
// Created by shiyan on 25-2-28.
//

#include "../include/adjacency_table.hpp"

adjacency_table::adjacency_table(const int _N): N(_N), table(_N, std::unordered_map<int, int>()) {}
adjacency_table::~adjacency_table() {}

void adjacency_table::merge(const adjacency_table &y)
{
  for (int i = 0; i < N; i++) {
    for (auto u : y.table[i]) {
      table[i][u.first] += u.second;
    }
  }
}