from ctypes import *
import os

my_dir = os.path.split(os.path.abspath(__file__))[0]
dll_path = os.path.join(os.path.split(my_dir)[0], r'c\bin\tstar.dll')

class Coordinate(Structure):
    _fields_ = [('x', c_int),
                ('y', c_int)]
    
tstar_dll = CDLL(dll_path)

# void* tstar_create();
_tstar_create = getattr(tstar_dll, "tstar_create")
_tstar_create.restype = c_void_p

# tstar_set_map(void* handler, int * map, int width, int height);
_tstar_set_map = getattr(tstar_dll, "tstar_set_map")
_tstar_set_map.argtypes = [c_void_p, c_void_p, c_int, c_int, Coordinate, Coordinate]
_tstar_set_map.restype = c_int

class TstarWrapper:
    def __init__(self):
        self._tstar = _tstar_create()
        
    def set_map(self, map_env, start, goal):
        _width = map_env.shape[1]
        _height = map_env.shape[0]
        _map = ((c_int * _height) * _width)()
        for i in range(_height):
            for j in range(_width):
                _map[i][j]= int(map_env[i][j])
        
        _start = Coordinate()
        _start.x = start[0]
        _start.y = start[1]
        
        _goal = Coordinate()
        _goal.x = goal[0]
        _goal.y = goal[1]
                
        res = _tstar_set_map(self._tstar, cast(_map,c_void_p), _width, _height, _start, _goal)
           
        return res