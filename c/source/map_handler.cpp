#include <cmath>
#include "map_handler.h"

Map::Map(int* map, int width, int height, Coordinate start, Coordinate goal) {
	m_width = width;
	m_height = height;
	m_start = start;
	m_goal = goal;

	int** _map = new int* [m_height];
	for (int i = 0; i < m_height; ++i)
		_map[i] = new int[m_width];

	for (int i = 0; i < m_height; i++) {
		for (int j = 0; j < m_width; j++) {
			_map[i][j] =  *((map + i * width) + j);
		}
	}
	m_map = _map;

	/*
	cout << "w,h,s,g " << m_width << ", " << m_height << ", ";
	cout << "(" << m_start.x << "," << m_start.y << ") ";
	cout << "(" << m_goal.x << "," << m_goal.y << ")" << endl;
	
	for (int i=0; i < m_height; i++) {
		for (int j = 0; j < m_width; j++) {
			cout << m_map[i][j] << ",";
		}
		cout << endl;
	}
	*/
}

int Map::valid_cell(Coordinate cell) {
	if (cell.x < 0 || cell.x >= m_width || cell.y < 0 || cell.y >= m_height)
		return 0;
	if (m_map[cell.y][cell.x] == 1)
		return 0;
	return 1;
}

int Map::valid_edge(Coordinate cell1, Coordinate cell2) {
	if (!valid_cell(cell1) || !valid_cell(cell2))
		return 0;
	vector<double> v = { (double)cell2.x - cell1.x , (double)cell2.y - cell1.y};
	double v_norm = l2_norm(v);
	double x1 = double(cell1.x) + 0.5;
	double y1 = double(cell1.y) + 0.5;
	double x2 = double(cell2.x) + 0.5;
	double y2 = double(cell2.y) + 0.5;
	int count = 1;
	double step_size = 0.3;

	while (double(count) * step_size < v_norm) {
		double a = double(count) * step_size / v_norm;
		Coordinate check;
		check.x = (1.0 - a) * x1 + a * x2;
		check.y = (1.0 - a) * y1 + a * y2;
		if (!valid_cell(check)) {
			return 0;
		}
		count++;
	}
	return 1;
}

double Map::l2_norm(std::vector<double> const& u) {
	double accum = 0.;
	for (int i = 0; i < u.size(); ++i) {
		accum += u[i] * u[i];
	}
	return sqrt(accum);
}

Coordinate Map::get_start() {
	return m_start;
}

Coordinate Map::get_goal() {
	return m_goal;
}