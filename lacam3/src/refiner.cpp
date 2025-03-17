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
      std::max(1, std::min(get_random_int(MT, 1, 10), int(N / 4))); // 一组agents的个数
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
      if (hostile(i, j, ins->G->width, ins->G->height, ins->starts[i], ins->goals[i], ins->starts[j], ins->goals[j], 1)) {
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

// refine solutions for given group of agents
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
                const int verbose, const int f)
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
      std::max(1, std::min(get_random_int(MT, 1, 10), int(N / 4))); // 一组agents的个数
                                                                     //  std::cout << "num_refine_agents:\t" << num_refine_agents << std::endl;
  info(1, verbose, deadline, "refinerRRGroup-", seed,
       "\tsize of modif set: ", num_refine_agents);
  Paths new_paths(N);
  std::vector<int> group;
  for (int _i = 0; _i < N - 1; _i++) {
    const int i = order[_i];
    if (updated[i]) continue;
    group.push_back(i);

    for (int _j = _i + 1; _j < N; _j++) {
      const int j = order[_j];
      if (updated[j]) continue;
      if (hostile(i, j, ins->G->width, ins->G->height, ins->starts[i], ins->goals[i], ins->starts[j], ins->goals[j], f))
        group.push_back(j);
      if (group.size() == num_refine_agents)
        break;
    }
//    std::cout << "group size: " << group.size() << '\n';
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
void xflip(P &si, P &sj, P &ei, P &ej, int &bix, int &bjx, const int W, const int H)
{
  si.first = W - si.first - 1;
  sj.first = W - sj.first - 1;
  ei.first = W - ei.first - 1;
  ej.first = W - ej.first - 1;
  bix = -bix; bjx = -bjx;
}
void yflip(P &si, P &sj, P &ei, P &ej, int &biy, int &bjy, const int W, const int H)
{
  si.second = H - si.second - 1;
  sj.second = H - sj.second - 1;
  ei.second = H - ei.second - 1;
  ej.second = H - ej.second - 1;
  biy = -biy; bjy = -bjy;
}
void xyflip(P &si, P &sj, P &ei, P &ej, int &bix, int &biy, int &bjx, int &bjy, int &W, int &H)
{
  std::swap(si.first, si.second);
  std::swap(sj.first, sj.second);
  std::swap(ei.first, ei.second);
  std::swap(ej.first, ej.second);
  std::swap(bix, biy);  std::swap(bjx, bjy);
  std::swap(W, H);
}
bool inD(const P u, const P s, const P e)
{
  return (u.first >= std::min(s.first, e.first) && u.first <= std::max(s.first, e.first)
          && u.second >= std::min(s.second, e.second) && u.second <= std::max(s.second, e.second));
}

bool inDij(const P u, const P si, const P ei, const P sj, const P ej)
{
  return inD(u, si, ei) && inD(u, sj, ej);
}
bool terminal_conflict(const P si, const P ei, const P sj, const P ej)
{
  return ((inDij(ei, si, ei, sj, ej) && dis(si, ei) <= dis(sj, ei))
          || (inDij(ej, si, ei, sj, ej) && dis(sj, ej) <= dis(si, ej)));
}
// f 粗化系数
bool hostile(const int i, const int j, int W, int H, Vertex *_si, Vertex *_ei, Vertex *_sj, Vertex *_ej, const int f)
{
  auto xis = _si->x / f, xie = _ei->x / f, xjs = _sj->x / f, xje= _ej->x / f;
  auto yis = _si->y / f, yie = _ei->y / f, yjs = _sj->y / f, yje = _ej->y / f;
  P si = std::make_pair(xis, yis), sj = std::make_pair(xjs, yjs);
  P ei = std::make_pair(xie, yie), ej = std::make_pair(xje, yje);
  int bix = sign(xie-xis), biy = sign(yie - yis), bjx = sign(xje - xjs), bjy = sign(yje - yjs); // 行进方向
  int Bi = std::abs(bix) + std::abs(biy), Bj = std::abs(bjx) + std::abs(bjy);
  int B = Bi + Bj;
  W /= f; H /= f;

  if (si == sj && ei == ej)
    return true;

  // 活动区域的边界
  int Dix1 = std::min(xis, xie), Dix2 = std::max(xis, xie);
  int Diy1 = std::min(yis, yie), Diy2 = std::max(yis, yie);
  int Djx1 = std::min(xjs, xje), Djx2 = std::max(xjs, xje);
  int Djy1 = std::min(yjs, yje), Djy2 = std::max(yjs, yje);

  // Dij is empty
  if (Dix1 > Djx2 || Djx1 > Dix2 || Diy1 > Djy2 || Djy1 > Diy2)
    return false;

  if (B == 2) {
    // suppose that bix = 1, biy = 0
    if (bix == 0)
      xyflip(si, sj, ei, ej, bix, biy, bjx, bjy, W, H);
    if (bix == -1)
      xflip(si, sj, ei, ej, bix, bjx, W, H);
    // 根据j的行进方向展开讨论
    if (bjx == 1 && bjy == 0) {
      if (terminal_conflict(si, ei, sj, ej)) // terminal conflict
        return true;
    }
    if (bjx == -1 && bjy == 0)
      return true;
    if (bjx == 0 && (bjy == 1 || bjy == -1)) {
      if (std::abs(xis - xjs) == std::abs(yis - yjs))
        return true;
      if (terminal_conflict(si, ei, sj, ej))
        return true;
    }
  }
  if (B == 3) {
    if (Bi == 1) {
      std::swap(si, sj);  std::swap(ei, ej);
      std::swap(Bi, Bj);
      std::swap(bix, bjx);  std::swap(biy, bjy);
    } // then we have Bi = 2
    if (bix == -1)
      xflip(si, sj, ei, ej, bix, bjx, W, H);
    if (biy == -1)
      yflip(si, sj, ei, ej, biy, bjy, W, H);
    // then we have bix = biy = 1

    // case 2.1
    if (bjx == 1 && bjy == 0) {
      if (xis + yis == xjs + yjs)
        return true; // 敌对 or 兼容
    }
    // case 2.3
    if (bjx == 0 && bjy == 1) {
      if (xis + yis == xjs + yjs)
        return true; // 敌对 or 兼容
    }
  }
    if (B == 4) {
      if (bix == -1)
        xflip(si, sj, ei, ej, bix, bjx, W, H);
      if (biy == -1)
        yflip(si, sj, ei, ej, biy, bjy, W, H);
      // case 3.1
      if (bjx == 1 && bjy == 1) {
        if (xis + yis == xjs + yjs)
          return true;
      }
    }
  return false;
}
void test_hostile(const Instance *ins, const int f)
{
  const auto N = ins->N;
  for (int i = 0; i < N; i++)
      for (int j = i + 1; j < N; j++)
        if (hostile(i, j, ins->G->width, ins->G->height, ins->starts[i], ins->goals[i], ins->starts[j], ins->goals[j], 1)) {
          std::cout << "ai: " << i << ", aj: "<< j << '\n';
          std::cout << "si: " << "(" << ins->starts[i]->x << "," << ins->starts[i]->y << ")" << '\n';
          std::cout << "ei: " << "(" << ins->goals[i]->x << "," << ins->goals[i]->y << ")" << '\n';
          std::cout << "sj: " << "(" << ins->starts[j]->x << "," << ins->starts[j]->y << ")" << '\n';
          std::cout << "ej: " << "(" << ins->goals[j]->x << "," << ins->goals[j]->y << ")" << '\n';
        }
}
