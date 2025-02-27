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

typedef std::pair<int, int> P;
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
                const int verbose = 0, const int f = 2);

int sign(const int x);

int dis(const P u, const P v);

void xflip(P &si, P &sj, P &ei, P &ej, int &bix, int &bjx, const int W, const int H);
void yflip(P &si, P &sj, P &ei, P &ej, int &biy, int &bjy, const int W, const int H);
void xyflip(P &si, P &sj, P &ei, P &ej, int &bix, int &biy, int &bjx, int &bjy, int &W, int &H);
bool inD(const P u, const P s, const P e);
bool inDij(const P u, const P si, const P ei, const P sj, const P ej);
bool terminal_conflict(const P si, const P ei, const P sj, const P ej);
bool hostile(const int i, const int j, int W, int H, Vertex *_si, Vertex *_ei, Vertex *_sj, Vertex *_ej, const int f);
void test_hostile(const Instance *ins, const int f);