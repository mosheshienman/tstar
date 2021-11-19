#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include <vector>

using namespace std;

typedef struct Coordinate {
	int x;
	int y;
} Coordinate;

class Map {
	public:
		Map(int* map, int width, int height, Coordinate start, Coordinate goal);
		
		int valid_cell(Coordinate cell);
		int valid_edge(Coordinate cell1, Coordinate cell2);
		Coordinate get_start();
		Coordinate get_goal();

	private:
		Coordinate m_start;
		Coordinate m_goal;
		int m_width;
		int m_height;
		int** m_map = 0;

		double l2_norm(std::vector<double> const& v);
};

#endif // MAP_HANDLER_H