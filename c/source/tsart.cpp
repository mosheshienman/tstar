#include "tstar.h"

extern "C" {

void* tstar_create() {
	TStarImp* handler = new TStarImp;
	return (void*)handler;
}
int tstar_set_map(void* handler, int* map, int width, int height, Coordinate start, Coordinate goal) {
	TStarImp* tstar = (TStarImp*)handler;

	int res = tstar->set_map(map, width, height, start, goal);
	return res;
}

} // extern "C"