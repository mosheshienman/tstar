#include <iostream>
#include <vector>
#include <cmath>
#include<bits/stdc++.h>
#include <stdio.h>
using namespace std;

#define ROW 10
#define COL 9

// In map: 0 = collision free   1 = collision

typedef pair<int, int> Pair;
typedef pair<double, Pair> sortedPairs;

struct cell
{
    int parentI, parentJ;
    // f = g + h
    double g,h,f;
};
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
bool isGoal(int row, int col, Pair goal)
{
    return ((row == goal.first) && (col == goal.second));
}

double heuristic(Pair cell, Pair goal)
{
    // Euclidean distance
    double h;
//    h = sqrt((cell.first - goal.first)*(cell.first - goal.first)
//            + (cell.second-goal.second)*(cell.second-goal.second));
    h = sqrt(pow((cell.first - goal.first),2) + pow((cell.second-goal.second),2));
    return h;
}

void AStarSearch(int map[][COL], Pair start, Pair goal)
{
    if (!isValid(start.first, start.second) || !isValid(goal.first, goal.second))
    {
        printf("start or goal states out of range\n");
        return;
    }
    if (!collisionFree(map, start.first, start.second) || !collisionFree(map, goal.first, goal.second))
    {
        printf("start or goal states are not collision free\n");
        return;
    }
    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    cell cellDetails[ROW][COL];
    int i, j, k, l;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parentI = -1;
            cellDetails[i][j].parentJ = -1;
        }
    }
    // Initialising the parameters of the starting state
    i = start.first, j = start.second;
    cellDetails[i][j].f = heuristic(start, goal);
    cellDetails[i][j].h = cellDetails[i][j].f;
    cellDetails[i][j].g = 0;
    cellDetails[i][j].parentI = i;
    cellDetails[i][j].parentJ = j;
    /*
    Create an open list having information as-
    <f, <x, y, theta, velocity>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implemented as a set of pair of states.*/
    set <sortedPairs> openList;

    // Put the starting state on the open list and set its
    // 'f' as the heuristic cost
    openList.insert(make_pair(heuristic(start, goal),
                              make_pair(start.first, start.second)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        sortedPairs s = *openList.begin();
        i = s.second.first;
        j = s.second.second;

        if (isGoal(i, j, goal)) {
            // reconstruct the path
            //int i = goal.i, j = goal.j, th = goal.th, v = goal.v;
            int iParent, jParent;
            printf("The path from goal to start:\n");
            printf("[%d, %d]\n", i, j);
            while (!((i == start.first) && (j == start.second))) {
                iParent = cellDetails[i][j].parentI;
                jParent = cellDetails[i][j].parentJ;
                printf("[%d, %d]\n", iParent, jParent);
                i = iParent;
                j = jParent;
            }
            /*absTheta = (double) th * (2 * M_PI) / robot.getNumOfOrientation();
            printf("[%d, %d, %.4f, %d]\n", i, j, absTheta, v);*/
            return;
        }

        Pair current{i, j};

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        closedList[i][j] = true;

        // find all the legal neighbors
        int potentialNeighbors[8][2] =
                {
                        {-1, -1},
                        {0,  -1},
                        {1,  -1},
                        {1,  0},
                        {1,  1},
                        {0,  1},
                        {-1, 1},
                        {-1, 0}
                };
        int neighbors[8][2] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
        int numOfNeighbors = 0;
        for (int ind = 0; ind < 8; ind++) {
            int tempX = i + potentialNeighbors[ind][0];
            int tempY = j + potentialNeighbors[ind][1];
            if ((isValid(tempX, tempY)) && collisionFree(map, tempX, tempY)) {
                neighbors[numOfNeighbors][0] = tempX;
                neighbors[numOfNeighbors][1] = tempY;
                numOfNeighbors++;
            }
        }

        double gNew, hNew, fNew;

        // update f score for the neighbors
        k = 0;
        int tempI, tempJ;
        while ((neighbors[k][0] != -1) && (k < 8))
        {
            tempI = neighbors[k][0];
            tempJ = neighbors[k][1];
            Pair localState{tempI, tempJ};
            // V1. Do not search in the closed list
            if (closedList[tempI][tempJ])
            {
                k++;
                continue;
            }

            gNew = cellDetails[i][j].g + heuristic(current, localState);
            hNew = heuristic(localState, goal);
            fNew = gNew + hNew;
            if ((cellDetails[tempI][tempJ].f == FLT_MAX)
                || (cellDetails[tempI][tempJ].f > fNew))
            {
                openList.insert(make_pair(fNew,
                                          make_pair(tempI, tempJ)));
                // Update the details of this state
                cellDetails[tempI][tempJ].parentI = i;
                cellDetails[tempI][tempJ].parentJ = j;
                cellDetails[tempI][tempJ].g = gNew;
                cellDetails[tempI][tempJ].h = hNew;
                cellDetails[tempI][tempJ].f = fNew;
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
    Pair start{4, 3};
    Pair goal{2, 1};
    AStarSearch(map, start, goal);
    //std::cout << "Hello, World!" << std::endl;
    return 0;
}
