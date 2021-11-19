#include <queue>
#include "astar_imp.h"

int AStarImp::run(Map* map) {
	Coordinate start = map->get_start();
	Coordinate goal = map->get_goal();

	std::priority_queue<State> open;

}