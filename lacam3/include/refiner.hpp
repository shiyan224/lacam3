/*
 * Implementation of refiners
 *
 * references:
 * Iterative Refinement for Real-Time Multi-Robot Path Planning.
 * Keisuke Okumura, Yasumasa Tamura, and Xavier Défago.
 * In Proceedings of IEEE/RSJ International Conference on Intelligent Robots and
 * Systems (IROS). 2021.
 *
 * Anytime multi-agent path finding via large neighborhood search.
 * Jiaoyang Li, Zhe Chen, Daniel Harabor, P Stuckey, and Sven Koenig.
 * In Proceedings of International Joint Conference on Artificial Intelligence
 * (IJCAI). 2021.
 */

#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "metrics.hpp"
#include "sipp.hpp"
#include "translator.hpp"
#include "utils.hpp"

Solution refine(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed = 0,
                const int verbose = 0);

Solution refineRR(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed = 0,
                const int verbose = 0);

bool refineGroup(std::vector<int> &group, const Instance *ins, const Deadline *deadline,
                 Paths &paths, Paths &new_paths, DistTable *D, const int seed, CollisionTable *CT);

Solution refineRRGroup(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed = 0,
                const int verbose = 0);

int sign(const int x);

bool hostile(const int i, const int j, const Instance *ins);