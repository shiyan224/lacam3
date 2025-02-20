#include "../include/refiner.hpp"

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

Solution refineRR(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed,
                const int verbose)
{
  if (solution.empty()) return Solution();
  info(0, verbose, deadline, "refinerRR-", seed, "\tactivated");
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
  std::vector<bool> updated(N, false);
//  if (is_expired(deadline)) return Solution();
  auto old_cost = 0;
  auto new_cost = 0;
  auto hostile_pairs = 0;
//  Paths new_paths(N);
  Path new_path_i, new_path_j;

  for (auto _i = 0; _i < N; _i++) {
    const auto i = order[_i];
    if (updated[i]) continue;
    for (auto _j = _i + 1; _j < N; _j++) {
      const auto j = order[_j];
      if (updated[j]) continue;
      if (hostile(i, j, ins)) {
        hostile_pairs++;
        old_cost = new_cost = 0;
        // compute old cost
        old_cost += get_path_loss(paths[i]) + get_path_loss(paths[j]);
        CT.clearPath(i, paths[i]); CT.clearPath(j, paths[j]);

        // re-planning
        new_path_i = sipp(i, ins->starts[i], ins->goals[i], D, &CT, deadline,
                                   old_cost - new_cost - 1);
        if (new_path_i.empty()) {
//          std::cout << "faile to refine i!" << '\n';
          CT.enrollPath(i, paths[i]); CT.enrollPath(j, paths[j]);
          continue;
        }
        else {
          new_cost += get_path_loss(new_path_i);
          CT.enrollPath(i, new_path_i);
        }
        new_path_j = sipp(j, ins->starts[j], ins->goals[j], D, &CT, deadline,
                             old_cost - new_cost - 1);
        // success
        if (!new_path_j.empty()) {
          CT.enrollPath(j, new_path_j);
          paths[i] = new_path_i; paths[j] = new_path_j;
          updated[i] = updated[j] = true;
          break;
        } else { // fail
          CT.clearPath(i, new_path_i);
          CT.enrollPath(i, paths[i]); CT.enrollPath(j, paths[j]);
          continue;
        }
      }
    }
  }
  std::cout << "there are " << hostile_pairs << " pairs of hostile agents" << '\n';
  info(0, verbose, deadline, "refinerRR-", seed, "\tsum_of_loss: ", cost_before,
       " -> ", get_sum_of_loss_paths(paths));

  return translatePathsToConfigs(paths);
}

bool hostile(const int i, const int j, const Instance *ins)
{
  return true;
}
