#include <iostream>

#include "tstar_imp.h"

using namespace std;

int TStarImp::set_map(int* map, int width, int height, Coordinate start, Coordinate goal) {
	Map * _map = new Map(map, width, height, start, goal);
	m_map = _map;
	
	/*
	Coordinate c1;
	c1.x = 1;
	c1.y = 1;
	Coordinate c2;
	c2.x = 4;
	c2.y = 1;
	Coordinate c3;
	c3.x = 4;
	c3.y = 4;
	cout << "c1 is valid " << m_map->valid_cell(c1) << endl;
	cout << "c2 is valid " << m_map->valid_cell(c2) << endl;
	cout << "c3 is valid " << m_map->valid_cell(c3) << endl;
	cout << "c1-c2 is valid " << m_map->valid_edge(c1,c2) << endl;
	cout << "c1-c3 is not valid " << m_map->valid_edge(c1, c3) << endl;
	cout << "c2-c3 is valid " << m_map->valid_edge(c2, c3) << endl;
	*/
	return 1;
}