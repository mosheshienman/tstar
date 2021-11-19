#ifndef TSTAR_IMP_H
#define TSTAR_IMP_H

#include "map_handler.h"

class TStarImp {
	public:
		int set_map(int* map, int width, int height, Coordinate start, Coordinate goal);
	
	private:
		Map* m_map;
};

#endif // TSTAR_IMP_H