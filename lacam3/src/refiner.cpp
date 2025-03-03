#include "../include/refiner.hpp"

typedef std::pair<int,int> P;

Solution refine(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed,
                const int verbose)
{
  if (solution.empty()) return Solution();
  info(0, verbose, deadline, "refiner-", seed, "\tactivated");
  // setup
  const auto N = ins->N;
  auto MT = std::mt19937(seed);
  auto paths = translateConfigsToPaths(solution);
  auto cost_before = get_sum_of_loss_paths(paths);
  std::vector<int> order(N, 0);
  std::iota(order.begin(), order.end(), 0);
  auto CT = CollisionTable(ins);
  for (auto i = 0; i < N; ++i) CT.enrollPath(i, paths[i]);
  std::shuffle(order.begin(), order.end(), MT);

  const auto num_refine_agents =
      std::max(1, std::min(get_random_int(MT, 1, 30), int(N / 4))); // 一组agents的个数
//  std::cout << "num_refine_agents:\t" << num_refine_agents << std::endl;
  info(1, verbose, deadline, "refiner-", seed,
       "\tsize of modif set: ", num_refine_agents);
  for (auto k = 0; (k + 1) * num_refine_agents < N; ++k) {
    if (is_expired(deadline)) return Solution();
    //std::cout << "group:\t" << k << std::endl;
    auto old_cost = 0;
    auto new_cost = 0;

    // compute old cost
    for (auto _i = 0; _i < num_refine_agents; ++_i) {
      const auto i = order[k * num_refine_agents + _i];
      old_cost += get_path_loss(paths[i]);
      CT.clearPath(i, paths[i]);
    }
    //std::cout << "old_cost:\t" << old_cost << std::endl;
    // re-planning
    Paths new_paths(num_refine_agents);
    for (auto _i = 0; _i < num_refine_agents; ++_i) {
      const auto i = order[k * num_refine_agents + _i];
      // note: I also tested A*, but SIPP was better
      new_paths[_i] = sipp(i, ins->starts[i], ins->goals[i], D, &CT, deadline,
                           old_cost - new_cost - 1);
      if (new_paths[_i].empty()) break;  // failure
      new_cost += get_path_loss(new_paths[_i]);
      CT.enrollPath(i, new_paths[_i]);
    }

    if (!new_paths[num_refine_agents - 1].empty() && new_cost <= old_cost) {
      // success
//      std::cout << "refine success!" << std::endl;
      for (auto _i = 0; _i < num_refine_agents; ++_i) {
        const auto i = order[k * num_refine_agents + _i];
        paths[i] = new_paths[_i];
      }
    } else {
      // failure
//      std::cout << "refine fail!" << std::endl;
      for (auto _i = 0; _i < num_refine_agents; ++_i) {
        const auto i = order[k * num_refine_agents + _i];
        if (!new_paths[_i].empty()) CT.clearPath(i, new_paths[_i]);
        CT.enrollPath(i, paths[i]);
      }
    }
  }

  info(0, verbose, deadline, "refiner-", seed, "\tsum_of_loss: ", cost_before,
       " -> ", get_sum_of_loss_paths(paths));

  return translatePathsToConfigs(paths);
}
// refine solutions for a given group of agents
bool refineGroup(std::vector<int> &group, const Instance *ins, const Deadline *deadline,
                 Paths &paths, Paths &new_paths, DistTable *D, const int seed, CollisionTable *CT)
{
  auto old_cost = 0, new_cost = 0;

  // get old cost
  for (auto i : group) {
    old_cost += get_path_loss(paths[i]);
    CT->clearPath(i, paths[i]);
  }

  // re-planning
  bool success = true;
  for (auto i : group) {
    new_paths[i] = sipp(i, ins->starts[i], ins->goals[i], D, CT, deadline,
                        old_cost - new_cost - 1);
    if (new_paths[i].empty()) {
      success = false;
      break;
    }
    new_cost += get_path_loss(new_paths[i]);
    CT->enrollPath(i, new_paths[i]);
  }
  if (success) {
    for (auto i : group)
      paths[i] = new_paths[i];
    return true;
  }
  else {
    for (auto i : group) {
      if (!new_paths[i].empty()) CT->clearPath(i, new_paths[i]);
      CT->enrollPath(i, paths[i]);
    }
    return false;
  }
}

Solution refineCompromiseNumber(const Instance *ins, adjacency_table &CG, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed,
                const int verbose)
{
  if (solution.empty()) return Solution();
  // setup
  const auto N = ins->N;
  auto MT = std::mt19937(seed);
  auto paths = translateConfigsToPaths(solution);
  std::vector<int> order(N, 0);
  std::iota(order.begin(), order.end(), 0);
  auto CT = CollisionTable(ins);
  for (auto i = 0; i < N; ++i) CT.enrollPath(i, paths[i]);
  std::shuffle(order.begin(), order.end(), MT);

  const auto num_refine_agents =
      std::max(1, std::min(get_random_int(MT, 1, 30), int(N / 4))); // 一组agents的个数

  // 匹配：贪心一下
  auto table = CG.table;
  using Edge = std::tuple<int, int, int>;
  std::vector<Edge> edge;
  for (int i = 0; i < N; i++) {
    for (auto e:table[i]) { // (i, e.first, e.second)
      edge.emplace_back(Edge(i, e.first, e.second));
//      std::cout << i << ' ' << e.first << ' ' << e.second << '\n';
    }
  }

  // 按边权降序排列
  std::sort(edge.begin(), edge.end(), [&](Edge &u, Edge &v){
    return std::get<2>(u) > std::get<2>(v);
  });
  std::vector<bool> matched(N, false);
  Paths new_paths(N);
  for (auto &e:edge) {
    auto i = std::get<0>(e), j = std::get<1>(e);
    if (!matched[i] && !matched[j]) {
      std::vector<int> group = {i, j};
      if (refineGroup(group, ins, deadline, paths, new_paths, D, seed, &CT)) {
        matched[i] = matched[j] = true;
      }
    }
  }

  // 剩下的随机
  std::vector<int> group;
  for (int _i = 0; _i < N; _i++) {
    auto i = order[_i];
    if (!matched[i])
      group.emplace_back(i);
    if (_i == N - 1 || group.size() == num_refine_agents) {
      if (refineGroup(group, ins, deadline, paths, new_paths, D, seed, &CT)) {
        for (auto j : group)
          matched[j] = true;
      }
      group.clear();
    }
  }
  return translatePathsToConfigs(paths);
}

