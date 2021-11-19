#include "map_handler.h"

typedef struct State {
	Coordinate coordinate;
	double theta;
	double speed;

	double g_value;
	double f_value;

	State* parent;
} State;