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

// 每次提取一对敌对agents进行refine
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

Solution refineRRGroup(const Instance *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed,
                const int verbose)
{
  if (solution.empty()) return Solution();
  info(0, verbose, deadline, "refinerRRGroup-", seed, "\tactivated");
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
  const auto num_refine_agents =
      std::max(1, std::min(get_random_int(MT, 1, 30), int(N / 4))); // 一组agents的个数
                                                                     //  std::cout << "num_refine_agents:\t" << num_refine_agents << std::endl;
  info(1, verbose, deadline, "refinerRRGroup-", seed,
       "\tsize of modif set: ", num_refine_agents);
  Paths new_paths(N);
  std::vector<int> group;
  for (int _i = 0; _i < N - 1; _i++) {
    const int i = order[_i];
    if (updated[i]) continue;
    for (int _j = _i + 1; _j < N; _j++) {
      const int j = order[_j];
      if (updated[j]) continue;
      if (hostile(i, j, ins))
        group.push_back(j);
      if (group.size() == num_refine_agents)
        break;
    }

    if (!group.empty() && refineGroup(group, ins, deadline, paths, new_paths, D, seed, &CT)) {
      for (auto v : group)
        updated[v] = true;
    }
    group.clear();
  }
  for (int _i = 0; _i < N; _i++) {
    const int i = order[_i];
    if (updated[i]) continue;
    group.push_back(i);
    if (!group.empty() && (group.size() == num_refine_agents || _i == N - 1)) {
      refineGroup(group, ins, deadline, paths, new_paths, D, seed, &CT);
    }
    group.clear();
  }
  info(0, verbose, deadline, "refinerRRGroup-", seed, "\tsum_of_loss: ", cost_before,
       " -> ", get_sum_of_loss_paths(paths));

  return translatePathsToConfigs(paths);
}
int sign(const int x)
{
  if (x > 0)
    return 1;
  if (x == 0)
    return 0;
  if (x < 0)
    return -1;
}
int dis(const P u, const P v)
{
  return std::abs(u.first - v.first) + std::abs(u.second - v.second);
}
// 在粗化图上对agents分组，再进行refine
void xflip(P &u, const int W, const int H)
{
  u.first = W - u.first - 1;
}
void yflip(P &u, const int W, const int H)
{
  u.second = H - u.second - 1;
}
void xyflip(P &u)
{
  std::swap(u.first, u.second);
}
bool hostile(const int i, const int j, const Instance *ins)
{
  const auto N = ins->N;
  auto _si = ins->starts[i], _sj = ins->starts[j], _ei = ins->goals[i], _ej = ins->goals[j];
  auto xis = _si->x, xie = _ei->x, xjs = _sj->x, xje= _ej->x;
  auto yis = _si->y, yie = _ei->y, yjs = _sj->y, yje = _ej->y;

  P si = std::make_pair(xis, yis), sj = std::make_pair(xjs, yjs);
  P ei = std::make_pair(xie, yie), ej = std::make_pair(xje, yje);
  auto W = ins->G->width, H = ins->G->height;

  int bix = sign(xie-xis), biy = sign(yie - yis), bjx = sign(xje - xjs), bjy = sign(yje - yjs);
  int Bi = std::abs(bix) + std::abs(biy), Bj = std::abs(bjx) + std::abs(bjy);
  int B = Bi + Bj;

  if (B == 2) {
    // suppose that bix = 1, biy = 0
    if (bix == 0)
      xyflip()
  }
  else if (B == 3)
    else if (B == 4)
  return true;
}
