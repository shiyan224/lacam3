#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "adjacency_table.hpp"
#include "utils.hpp"

Solution solve(const Instance &ins, adjacency_table &CG, const int verbose = 0,
               const Deadline *deadline = nullptr, int seed = 0);
