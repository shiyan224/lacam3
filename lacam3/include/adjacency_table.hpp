//
// Created by shiyan on 25-2-28.
//

#ifndef LACAM_PROJECT_ADJACENCY_TABLE_HPP
#define LACAM_PROJECT_ADJACENCY_TABLE_HPP

#include "utils.hpp"

using Adj_table = std::vector<std::unordered_map<int, int>>;
class adjacency_table
{
public:
  Adj_table table;
  const int N;
  adjacency_table(const int _N);
  ~adjacency_table();
  void merge(const adjacency_table &y);
private:
};

#endif  // LACAM_PROJECT_ADJACENCY_TABLE_HPP
