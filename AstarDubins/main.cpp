#include <iostream>
#include <vector>
#include <cmath>
//#include<bits/stdc++.h>
#include <cstring>
#include <cfloat>
#include <set>
#include <tuple>
#include <vector>
using namespace std;

#include "Robot.cpp"

#define ROW 10
#define COL 9

// In map: 0 = collision free   1 = collision

/*template <typename T1, typename T2, typename T3, typename T4> struct state;

template <typename T1, typename T2, typename T3, typename T4>
        state<T1, T2, T3, T4> make_state(int x, int y, int th, int v)
{
    return (state<T1, T2, T3, T4> (x, y, th, v));
}*/
typedef tuple<int, int, int, int> state;
typedef pair<double, state> sortedStates;

struct State
{
    // define the state i,j for x,y coordinate th for theta v for velocity
    // 0 <= theta <= number of orientation -1
    // actual theta calculate by: theta *  number of orientation / 2 PI
    int i, j, th, v;
};
struct StateParam
{
    int parent_i, parent_j, parent_th, parent_v;
    // f = g + h
    double f, g, h;
    int pathFromParent[6] = {}; // 5 optional segments and last value for type of the path
};

double* vanilaDubinsPath(Robot robot, State s, State g, double r);
double* dubinsLSL(double array[3], double alpha, double beta, double d);
double* dubinsLSR(double array[3], double alpha, double beta, double d);
double* dubinsRSL(double array[3], double alpha, double beta, double d);
double* dubinsRSR(double array[3], double alpha, double beta, double d);
double* dubinsRLR(double array[3], double alpha, double beta, double d);
double* dubinsLRL(double array[3], double alpha, double beta, double d);
double* sendAndReturn(double arr[3]);
double dubinsLength(double path[6], double r);
bool pathValidityCheck(Robot robot, int map[][COL], State s, double path[4], double r, double stepSize);
bool isValid(int row, int col);
bool collisionFree(int map[][COL], int row, int col);
double* obstacleDubinsPath(Robot robot, int map[][COL], State s, State g, double r);
double* dubinsPathSample(Robot robot, State s, double path[4], double t, double r);
double* dubinsSegment(double param, double segInit[3], int type);
void AStarSearch(Robot robot, int map[][COL], State start, State goal);
double twoPIMod(double a);

//check if cell in the map limits
bool isValid(int row, int col)
{
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

//check if cell is collision free
bool collisionFree(int map[][COL], int row, int col)
{
    return map[row][col] == 0;
}

//check if cell is goal
bool isGoal(int row, int col, int th, int v, State goal)
{
    return ((row == goal.i) && (col == goal.j) && (th == goal.th) && (v == goal.v));
}

double heuristic(Robot robot, State state, State goal)
{
    //double stateTh = (double) state.th / robot.getNumOfOrientation() * 2 *  M_PI;
    //double goalTh = (double) goal.th / robot.getNumOfOrientation() * 2 * M_PI;
    //double sInit[3] = {(double) state.i, (double) state.j, stateTh};
    //double sFinal[3] = {(double) goal.i, (double) goal.j, goalTh};
    double r = robot.getuMax() / robot.getvMin();
    double* path = vanilaDubinsPath(robot, state, goal, r);
    double length = dubinsLength(path, r);

    return (length / robot.getvMax());
}

double* vanilaDubinsPath(Robot robot, State s, State g, double r)
{
    double dx, dy, D, d, bestCost = -1, wayLength[6][3] = { }, theta, alpha, beta, array[3];
    double cost = 0, stepsize = 0.1;
    double bestPath[4] = {-1, -1, -1, -1};
    double* a;
    int i, j, bestWay = -1;

    //dx = g[0] - s[0];
    //dy = g[1] - s[1];
    //dx = g[1] - s[1];
    //dy = g[0] - s[0];
    dx = ((double) g.j - (double) s.j);
    dy = -((double) g.i - (double) s.i);
    //dx = (double) g.i - (double) s.i;
    //dy = (double) g.j - (double) s.j;
    D = sqrt(pow(dx, 2) + pow(dy, 2));
    d = D / r;
    if (r <= 0)
    {
        //std::cout<<"Turn Radius Must Be Positive";
        a = sendAndReturn(bestPath);
        return a;
    }

    double absThStart, absThGoal;
    absThStart = (double) s.th * (2 * M_PI) / robot.getNumOfOrientation();
    absThGoal = (double) g.th * (2 * M_PI) / robot.getNumOfOrientation();

    theta = twoPIMod((atan2(dy, dx))); // difference between stat and goal orientation
    alpha = twoPIMod((absThStart - theta)); // fix start orientation to the difference
    beta = twoPIMod((absThGoal - theta)); //fix goal orientation to the difference

    double* temparr = dubinsLSL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[0][i] = temparr[i];
    }
    temparr = dubinsLSR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[1][i] = temparr[i];
    }
    temparr = dubinsRSL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[2][i] = temparr[i];
    }
    temparr = dubinsRSR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[3][i] = temparr[i];
    }
    temparr = dubinsRLR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[4][i] = temparr[i];
    }
    temparr = dubinsLRL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[5][i] = temparr[i];
    }
    // find shortest path
    for(i = 0; i < 6; i++)
    {
        if (wayLength[i][0] != -1)
        {
            for (j = 0; j < 3; j++)
            {
                cost += wayLength[i][j];
            }
            if ((cost < bestCost) || (bestCost == -1))
            {
                bestWay = i;
                bestCost = cost;
            }
            cost = 0;
        }
    }
    if (bestWay == -1)
    {
        //std::cout<<"No Path";
        a = sendAndReturn(bestPath);
        return a;
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            bestPath[i] = wayLength[bestWay][i];
        }
        bestPath[3] = bestWay;
        /*
        if (bestWay == 0)
        {
            printf("dubins path type: LSL\n");
        } else if (bestWay == 1)
        {
            printf("dubins path type: LSR\n");
        }else if (bestWay == 2)
        {
            printf("dubins path type: RSL\n");
        }else if (bestWay == 3)
        {
            printf("dubins path type: RSR\n");
        }else if (bestWay == 4)
        {
            printf("dubins path type: RLR\n");
        }else if (bestWay == 5)
        {
            printf("dubins path type: LRL\n");
        }*/
        a = sendAndReturn(bestPath);
        return a;
    }
}

double* dubinsLSL(double array[3], double alpha, double beta, double d)
{
    double temp0, temp1, pSquared, t, p, q;
    int i;

    temp0 = d + sin(alpha) - sin(beta);
    pSquared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
    if (pSquared < 0)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        temp1 = atan2( (cos(beta)-cos(alpha)), temp0 );
        t = twoPIMod((-alpha + temp1 ));
        p = sqrt( pSquared );
        q = twoPIMod((beta - temp1 ));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* dubinsLSR(double array[3], double alpha, double beta, double d)
{
    double temp0, pSquared, t, p, q;
    int i;
    pSquared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha)+sin(beta)));
    if (pSquared < 0)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        p = sqrt(pSquared);
        temp0 = atan2((-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta))) - atan2(-2.0, p);
        t = twoPIMod((-alpha + temp0 ));
        q = twoPIMod((-twoPIMod(beta) + temp0));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* dubinsRSL(double array[3], double alpha, double beta, double d)
{
    double temp0, pSquared, t, p, q;
    int i;
    pSquared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha)+sin(beta)));
    if (pSquared < 0)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        p = sqrt( pSquared );
        temp0 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)) ) - atan2(2.0, p);
        t = twoPIMod((alpha - temp0));
        q = twoPIMod((beta - temp0));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* dubinsRSR(double array[3], double alpha, double beta, double d)
{
    double temp0,temp1,  pSquared, t, p, q;
    int i;
    temp0 = d-sin(alpha)+sin(beta);
    pSquared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta)-sin(alpha)));
    if (pSquared < 0)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        temp1 = atan2((cos(alpha)-cos(beta)), temp0);
        t = twoPIMod((alpha - temp1));
        p = sqrt(pSquared);
        q = twoPIMod((-beta + temp1));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* dubinsRLR(double array[3], double alpha, double beta, double d)
{
    double temp0, t, p, q;
    int i;
    temp0 = (6 - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha)-sin(beta))) / 8;
    if (fabs(temp0) > 1)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        p = twoPIMod((2 * M_PI - acos(temp0)));
        t = twoPIMod((alpha - atan2(cos(alpha)-cos(beta), d-sin(alpha)+sin(beta)) + twoPIMod(p/2)));
        q = twoPIMod((alpha - beta - t + twoPIMod(p)));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* dubinsLRL(double array[3], double alpha, double beta, double d)
{
    double temp0, t, p, q;
    int i;
    temp0 = (6 - d*d + 2*cos(alpha - beta) + 2*d*(-sin(alpha) + sin(beta))) / 8;
    if (fabs(temp0) > 1)
    {
        for (i = 0; i < 3; i++)
        {
            array[i] = -1;
        }
        return array;
    } else
    {
        p = twoPIMod((2 * M_PI - acos(temp0)));
        t = twoPIMod((-alpha - atan2(cos(alpha)-cos(beta), d + sin(alpha)-sin(beta)) + p/2));
        q = twoPIMod((twoPIMod(beta) - alpha -t + twoPIMod(p)));
        array[0] = fabs(t);
        array[1] = fabs(p);
        array[2] = fabs(q);
        return array;
    }
}

double* sendAndReturn(double arr[3])
{
    return arr;
}

double dubinsLength(double path[3], double r)
{
    double length = path[0];
    length += path[1];
    length += path[2];
    /*for (int i = 1; i < 5; i++)
    {
        length += path[i];
    }*/
    length *= r;
    return length;
}

bool pathValidityCheck(Robot robot, int map[][COL], State s, double path[4], double r, double stepSize)
{
    double x = 0;
    double length = dubinsLength(path, r);
    int temp = (int) floor(length / stepSize);
    int pathRows = (int) temp;
    int j, i = 0;
    double pathMid[pathRows][3] = {};
    // build all the mid points of the path
    while (x <= length) {
        double *tempPath = dubinsPathSample(robot, s, path, x, r);
        for (j = 0; j < 3; j++) {
            pathMid[i][j] = tempPath[j];
        }
        x += stepSize;
        i++;
    }
    // choose only the cells the path going throw
    double pathCells[pathRows][2] = {s.i, s.j};
    i = 1;
    for (j = 1; j < pathRows; j++) {
        int tempX = (int) ceil(pathMid[j][0]), tempY = (int) ceil(pathMid[j][1]);
        if ((tempX != pathCells[i - 1][0]) || (tempY != pathCells[i - 1][1]))
        {
            if (!collisionFree(map, tempX, tempY) || !isValid(tempX, tempY))
            {
                return false;
            }
            pathCells[i][0] = tempX;
            pathCells[i][1] = tempY;
            i ++;
        }
    }
    // pathCells[0] = i - 1; // the actual length of the array
    return true;
}

double* dubinsPathSample(Robot robot, State s, double path[4], double t, double r)
{
    double endPt[3] = {-1, -1, -1}, tPrime = t / r;
    double absThStart = (double) s.th * (2 * M_PI) / robot.getNumOfOrientation();
    double pInit[3] = {0, 0, absThStart};
    // L_SEG = 1    S_SEG = 2   R_SEG = 3
    int dirData[6][3] = {1, 2, 1, 1, 2, 3, 3, 2, 1, 3, 2, 3, 3, 1, 3, 1, 3, 1};
    int types[3] = {dirData[(int) path[3]][0], dirData[(int) path[3]][1], dirData[(int) path[3]][2]};
    double param1 = path[0], param2 = path[1];
    double* midPt1 = dubinsSegment(param1, pInit, types[0]);
    double midPt1Arr[3] = {midPt1[0], midPt1[1], midPt1[2]};
    double* midPt2 = dubinsSegment(param2, midPt1Arr, types[1]);
    double midPt2Arr[3] = {midPt2[0], midPt2[1], midPt2[2]};
    double * AndPt;

    if (tPrime < param1)
    {
        AndPt = dubinsSegment(tPrime, pInit, types[0]);
    } else if (tPrime < (param1 + param2))
    {
        AndPt = dubinsSegment(tPrime - param1, midPt1Arr, types[1]);
    } else
    {
        AndPt = dubinsSegment(tPrime - param1 - param2, midPt2Arr, types[2]);
    }
    endPt[0] = AndPt[0] * r + s.i;
    endPt[1] = AndPt[1] * r + s.j;
    endPt[2] =  twoPIMod(AndPt[2]);
    return sendAndReturn(endPt);
}

double* dubinsSegment(double param, double segInit[3], int type)
{
    double segEnd[3] = {};
    int L = 1, S = 2, R = 3;
    if (type == L)
    {
        segEnd[0] = segInit[0] + sin(segInit[2] + param) - sin(segInit[2]);
        segEnd[1] = segInit[1] - cos(segInit[2] + param) + cos(segInit[2]);
        segEnd[2] = segInit[2] + param;
    } else if (type == R)
    {
        segEnd[0] = segInit[0] - sin(segInit[2] - param) + sin(segInit[2]);
        segEnd[1] = segInit[1] + cos(segInit[2] - param) - cos(segInit[2]);
        segEnd[2] = segInit[2] - param;
    } else if (type == S)
    {
        segEnd[0] = segInit[0] + cos(segInit[2]) * param;
        segEnd[1] = segInit[1] + sin(segInit[2]) * param;
        segEnd[2] = segInit[2];
    }
    return sendAndReturn(segEnd);
}

double* obstacleDubinsPath(Robot robot, int map[][COL], State s, State g, double r)
{
    double dx, dy, D, d, bestCost = -1, wayLength[6][3] = { }, theta, alpha, beta, array[3];
    double cost = 0, stepsize = 0.1;
    double bestPath[6] = {-1, -1, -1, 0, 0, -1};
    double* a;
    int i, j, bestWay = -1;
    /*if ((g.i == 7) && (g.j == 3) && (g.th == 2))
    {
        int doron = 12;
    }*/
    //dx = (double) g.i - (double) s.i;
    //dy = (double) g.j - (double) s.j;
    dx = ((double) g.j - (double) s.j);
    dy = -((double) g.i - (double) s.i);
    D = sqrt(pow(dx, 2) + pow(dy, 2));
    d = D / r;
    if (r <= 0)
    {
        std::cout<<"Turn Radius Must Be Positive\n";
        a = sendAndReturn(bestPath);
        return a;
    }
    double absThStart, absThGoal;
    absThStart = (double) s.th * (2 * M_PI) / robot.getNumOfOrientation();
    absThGoal = (double) g.th * (2 * M_PI) / robot.getNumOfOrientation();

    /*double b = fmod(atan2(dy, dx), (-2 * M_PI));
    theta = fmod(atan2(dy, dx), (2 * M_PI)); // difference between stat and goal orientation
    alpha = fmod((absThStart-theta), (2 * M_PI)); // fix start orientation to the difference
    beta = fmod((absThGoal-theta), (2 * M_PI)); //fix goal orientation to the difference
     */
    theta = twoPIMod(atan2(dy, dx));
    alpha = twoPIMod((absThStart-theta));
    beta = twoPIMod((absThGoal-theta));

    double* temparr = dubinsLSL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[0][i] = temparr[i];
    }
    temparr = dubinsLSR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[1][i] = temparr[i];
    }
    temparr = dubinsRSL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[2][i] = temparr[i];
    }
    temparr = dubinsRSR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[3][i] = temparr[i];
    }
    temparr = dubinsRLR(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[4][i] = temparr[i];
    }
    temparr = dubinsLRL(array, alpha, beta, d);
    for (i = 0; i <3; i++)
    {
        wayLength[5][i] = temparr[i];
    }
    // find shortest path
    double tempPath[4] = {};
    for(i = 0; i < 6; i++)
    {
        if (wayLength[i][0] != -1)
        {
            tempPath[3] = i;
            for (j = 0; j < 3; j++)
            {
                cost += wayLength[i][j];
                tempPath[j] = wayLength[i][j];
            }
            if (((cost < bestCost) || (bestCost == -1)) &&
                pathValidityCheck(robot, map, s, tempPath, r, stepsize))
            {
                bestWay = i;
                bestCost = cost;
            }
            cost = 0;
        }
    }
    if (bestWay == -1)
    {
        //std::cout<<"No Path\n";
        a = sendAndReturn(bestPath);
        return a;
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            bestPath[i] = wayLength[bestWay][i];
        }
        bestPath[3] = bestWay;
        /*if (bestWay == 0)
        {
            printf("dubins path type: LSL\n");
        } else if (bestWay == 1)
        {
            printf("dubins path type: LSR\n");
        }else if (bestWay == 2)
        {
            printf("dubins path type: RSL\n");
        }else if (bestWay == 3)
        {
            printf("dubins path type: RSR\n");
        }else if (bestWay == 4)
        {
            printf("dubins path type: RLR\n");
        }else if (bestWay == 5)
        {
            printf("dubins path type: LRL\n");
        }*/
        a = sendAndReturn(bestPath);
        return a;
    }

}

double twoPIMod(double a)
{
    if (a < 0)
    {
        a = fmod(a, 2 * M_PI);
        return (a + 2 * M_PI);
    } else
    {
        return (fmod(a, 2 * M_PI));
    }

}

void AStarSearch(Robot robot, int map[][COL], State start, State goal)
{
    tuple <int, int, int, int> state;
    double r = 1;
    //Robot roboy2;
    if (!isValid(start.i, start.j) || !isValid(goal.i, goal.j))
    {
        printf("start or goal states out of range\n");
        return;
    }
    if (!collisionFree(map, start.i, start.j) || !collisionFree(map, goal.i, goal.j))
    {
        printf("start or goal states are not collision free\n");
        return;
    }
    int orient = robot.getNumOfOrientation(), numOfVelocity = 1;
    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL][orient][numOfVelocity];
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    StateParam stateDetails[ROW][COL][orient][numOfVelocity];
    int i, j, k, l;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            for (k = 0; k < orient; k++)
            {
                for (l = 0;  l < numOfVelocity; l++)
                {
                    stateDetails[i][j][k][l].f = FLT_MAX;
                    stateDetails[i][j][k][l].g = FLT_MAX;
                    stateDetails[i][j][k][l].h = FLT_MAX;
                    stateDetails[i][j][k][l].parent_i = -1;
                    stateDetails[i][j][k][l].parent_j = -1;
                    stateDetails[i][j][k][l].parent_th = -1;
                    stateDetails[i][j][k][l].parent_v = -1;
                }
            }
        }
    }
    // Initialising the parameters of the starting state
    i = start.i, j = start.j;
    int th = start.th, v = start.v;
    stateDetails[i][j][th][v].f = heuristic(robot, start, goal);
    stateDetails[i][j][th][v].h = stateDetails[i][j][th][v].f;
    stateDetails[i][j][th][v].g = 0;
    stateDetails[i][j][th][v].parent_i = i;
    stateDetails[i][j][th][v].parent_j = j;
    stateDetails[i][j][th][v].parent_th = th;
    stateDetails[i][j][th][v].parent_v = v;

    /*
    Create an open list having information as-
    <f, <x, y, theta, velocity>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of states.*/
    set <sortedStates> openList;

    // Put the starting state on the open list and set its
    // 'f' as the heuristic cost
    openList.insert(make_pair(heuristic(robot, start, goal),
            make_tuple (start.i, start.j, start.th, start.v)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        sortedStates s = *openList.begin();
        tuple<int, int, int, int> &tempTuple = s.second;
        i = get<0>(tempTuple);
        j = get<1>(tempTuple);
        th = get<2>(tempTuple);
        v = get<3>(tempTuple);

        if (isGoal(i, j, th, v, goal))
        {
            /*stateDetails[tempI][tempJ][tempTh][tempV].parent_i = i;
            stateDetails[tempI][tempJ][tempTh][tempV].parent_j = j;
            stateDetails[tempI][tempJ][tempTh][tempV].parent_th = th;
            stateDetails[tempI][tempJ][tempTh][tempV].parent_v = v;*/
            //getPath(robot, stateDetails, start, goal);

            // reconsruct the path
            //int i = goal.i, j = goal.j, th = goal.th, v = goal.v;
            int iParent, jParent, thParent, vParent;
            double absTheta = (double) th * (2 * M_PI) / robot.getNumOfOrientation();
            printf("The path from goal to start:\n");
            printf("[%d, %d, %.4f, %d]\n", i, j, absTheta, v);
            while (!((i == start.i) && (j == start.j)))
            {
                iParent = stateDetails[i][j][th][v].parent_i;
                jParent = stateDetails[i][j][th][v].parent_j;
                thParent = stateDetails[i][j][th][v].parent_th;
                vParent = stateDetails[i][j][th][v].parent_v;
                absTheta = (double) thParent * (2 * M_PI) / robot.getNumOfOrientation();
                printf("[%d, %d, %.4f, %d]\n", iParent, jParent, absTheta, vParent);
                i = iParent;
                j = jParent;
                th = thParent;
                v = vParent;
            }
            /*absTheta = (double) th * (2 * M_PI) / robot.getNumOfOrientation();
            printf("[%d, %d, %.4f, %d]\n", i, j, absTheta, v);*/
            return;
        }

        State current{i, j, th, v};

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        closedList[i][j][th][v] = true;

        int potentialNeighbors [8][2] =
                {
                    {-1,-1},{0,-1},{1,-1},{1,0},
                    {1,1},{0,1},{-1,1},{-1,0}
                };
        int neighbors[8][2] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
        int numOfNeighbors = 0;
        for (int ind = 0; ind < 8; ind++)
        {
            int tempX = i + potentialNeighbors[ind][0];
            int tempY = j + potentialNeighbors[ind][1];
            if ((isValid(tempX, tempY)) && collisionFree(map, tempX, tempY))
            {
                neighbors[numOfNeighbors][0] = tempX;
                neighbors[numOfNeighbors][1] = tempY;
                numOfNeighbors++;
            }
        }

        double gNew, hNew, fNew;

        k = 0;
        int tempI, tempJ;
        while ((neighbors[k][0] != -1) && (k < 8))
        {
            tempI = neighbors[k][0];
            tempJ = neighbors[k][1];
            for (int tempTh = 0; tempTh < orient; ++tempTh)
            {
                for (int tempV = 0; tempV < numOfVelocity ; ++tempV)
                {
                    State localState {tempI, tempJ, tempTh, tempV};
                    /*if (isGoal(tempI, tempJ, tempTh, tempV, goal))
                    {
                        stateDetails[tempI][tempJ][tempTh][tempV].parent_i = i;
                        stateDetails[tempI][tempJ][tempTh][tempV].parent_j = j;
                        stateDetails[tempI][tempJ][tempTh][tempV].parent_th = th;
                        stateDetails[tempI][tempJ][tempTh][tempV].parent_v = v;
                        //getPath(robot, stateDetails, start, goal);

                        // reconstruct the path
                        //int i = goal.i, j = goal.j, th = goal.th, v = goal.v;
                        int iParent, jParent, thParent, vParent;
                        double absTheta = (double) tempTh * (2 * M_PI) / robot.getNumOfOrientation();
                        printf("The path from goal to start:\n");
                        printf("[%d, %d, %.4f, %d]\n", tempI, tempJ, absTheta, tempV);
                        while ((i != start.i) && (j != start.j))
                        {
                            iParent = stateDetails[i][j][th][v].parent_i;
                            jParent = stateDetails[i][j][th][v].parent_j;
                            thParent = stateDetails[i][j][th][v].parent_th;
                            vParent = stateDetails[i][j][th][v].parent_v;
                            absTheta = (double) thParent * (2 * M_PI) / robot.getNumOfOrientation();
                            printf("[%d, %d, %.4f, %d]\n", iParent, jParent, absTheta, vParent);
                            i = iParent;
                            j = jParent;
                            th = thParent;
                            v = vParent;
                        }
                        absTheta = (double) th * (2 * M_PI) / robot.getNumOfOrientation();
                        printf("[%d, %d, %.4f, %d]\n", i, j, absTheta, v);
                        return;
                    }*/
                    // V1. Do not search in the closed list
                    if (closedList[tempI][tempJ][tempTh][tempV])
                    {
                        k++;
                        continue;
                    }
                    // calculate the dubins path from current state to temp state
                    // delx = tempI - i; delY = tempJ - j;
                    // delX and delY will use us when we will have transfer table between states
                    double* path = obstacleDubinsPath(robot, map, current, localState, r);
                    double localPath[6] = {path[0], path[1], path[2], path[3], path[4], path[5]};
                    // check there is a valid path between two states
                    if (localPath[0] != -1)
                    {
                        double localLength = dubinsLength(localPath, r);
                        gNew = stateDetails[i][j][th][v].g + localLength;
                        hNew = heuristic(robot, localState, goal);
                        fNew = gNew + hNew;
                        if ((stateDetails[tempI][tempJ][tempTh][tempV].f == FLT_MAX)
                            || (stateDetails[tempI][tempJ][tempTh][tempV].f > fNew))
                        {
                            openList.insert(make_pair(fNew,
                                                      make_tuple(tempI, tempJ, tempTh, tempV)));
                            // Update the details of this state
                            stateDetails[tempI][tempJ][tempTh][tempV].parent_i = i;
                            stateDetails[tempI][tempJ][tempTh][tempV].parent_j = j;
                            stateDetails[tempI][tempJ][tempTh][tempV].parent_th = th;
                            stateDetails[tempI][tempJ][tempTh][tempV].parent_v = v;
                            stateDetails[tempI][tempJ][tempTh][tempV].g = gNew;
                            stateDetails[tempI][tempJ][tempTh][tempV].h = hNew;
                            stateDetails[tempI][tempJ][tempTh][tempV].f = fNew;
                            for (int p = 0; p < 6; p++)
                            {
                                stateDetails[tempI][tempJ][tempTh][tempV].pathFromParent[p]
                                        = localPath[p];
                            }

                        }
                    }
                }
            }
            k++;
        }
    }
    printf("failed to find a path\n");
}

int main() {
    int map[ROW][COL] =
            {     // 0  1  2  3  4  5  6  7  8
                    {0, 0, 0, 1, 1, 0, 1, 0, 1}, // 0
                    {1, 1, 0, 0, 0, 0, 1, 0, 0}, // 1
                    {0, 0, 1, 0, 0, 0, 0, 0, 0}, // 2
                    {1, 1, 1, 0, 0, 0, 0, 0, 0}, // 3
                    {0, 0, 0, 0, 0, 0, 0, 0, 1}, // 4
                    {1, 1, 0, 0, 0, 0, 0, 0, 0}, // 5
                    {0, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
                    {1, 1, 1, 0, 0, 0, 0, 0, 0}, // 7
                    {0, 0, 0, 0, 0, 0, 1, 0, 0}, // 8
                    {1, 0, 0, 0, 1, 0, 0, 0, 0}  // 9
            };
    Robot robot1;

//    State start{2, 3, 0, 0};
//    State goal{2, 7, 0, 0};

//    State start{4, 3, 6, 0};
//    State goal{8, 3, 6, 0};

    State start{8, 8, 4, 0}; // (y, x, theta, v)
    State goal{1, 8, 0, 0};

//    State start{8, 3, 0, 0};
//    State goal{4, 0, 4, 0};

    // failed of finding a path
//    State start{9, 8, 2, 0}; // (y, x, theta, v)
//    State goal{2, 8, 2, 0};

    // without visualize
//    State start{2, 8, 6, 0}; // (y, x, theta, v)
//    State goal{9, 8, 6, 0};

    AStarSearch(robot1, map, start, goal);
    return 0;
}