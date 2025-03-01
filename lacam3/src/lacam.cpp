#include "../include/lacam.hpp"

Solution solve(const Instance &ins, adjacency_table &CG, int verbose, const Deadline *deadline,
               int seed)
{
  info(1, verbose, deadline, "pre-processing");
  auto planner = Planner(&ins, verbose, deadline, seed);
  return planner.solve(CG);
}
