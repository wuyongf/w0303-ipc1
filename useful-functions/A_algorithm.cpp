#include <fstream>
#include <iostream>
#include <sstream>

#include <string>
#include <vector>

#include <algorithm>

using std::cout;
using std::ifstream;
using std::istringstream;

using std::string;
using std::vector;

using std::abs;
using std::sort;

enum class State {kEmpty, kObstacle, kClosed,kPath,kStart,kFinish};

vector<State> ParseLine(string line) {
    istringstream sline(line);
    int n;
    char c;
    vector<State> row;
    while (sline >> n >> c && c == ',') {
        if (n == 0) {
            row.push_back(State::kEmpty);
        } else {
            row.push_back(State::kObstacle);
        }
    }
    return row;
}

vector<vector<State>> ReadBoardFile(const string &test) {

    ifstream myfile;
    string path1 = "/home/wyf/turtlebot_ws/src/cpp_tutorials/src/";
    string path = path1 + test;
    myfile.open(path);

    vector<vector<State>> board{};
    if (myfile) {
        string line;
        while (getline(myfile, line)) {
            vector<State> row = ParseLine(line);
            board.push_back(row);
        }
    }
    return board;
}

bool Compare(vector<int> v1, vector<int> v2)
{
    int f1 = v1[2] + v1[3];
    int f2 = v2[2] + v2[3];

    return f1 > f2;
}

/**
 * Sort the two-dimensional vector of ints in descending order.
 */
void CellSort(vector<vector<int>> *v) {
  sort(v->begin(), v->end(), Compare);
}

int Heuristic (const int& x1, const int& y1, const int& x2, const int& y2){
    int MD = abs(x2-x1)+abs(y2-y1);
    return MD;
}

// TODO: Write CheckValidCell here. Check that the
// cell is on the grid and not an obstacle (i.e. equals kEmpty).
bool CheckValidCell(int x,int y, vector<vector<State>>& grid){
  bool on_grid_x = (x >= 0 && x < grid.size());
  bool on_grid_y = (y >= 0 && y < grid[0].size());
  if (on_grid_x && on_grid_y)
    return grid[x][y] == State::kEmpty;
  return false;
}

void AddToOpen (int x, int y, int g, int h, vector<vector<int>>& open_nodes, vector<vector<State>>& grid){
    vector<int> node = {x,y,g,h};
    open_nodes.push_back(node);
    grid[x][y] = State::kClosed;
}

/**
 * Expand current nodes's neighbors and add them to the open list.
 */
// TODO: ExpandNeighbors(arguments) {
int ExpandNeighbors(vector<int>& current_node, vector<vector<int>>& open_nodes,
                     vector<vector<State>>& grid, const int (&goal)[2])
{
    // TODO: Get current node's data.
    int x = current_node[0];
    int y = current_node[1];
    int g = current_node[2];
    
    // TODO: Loop through current node's potential neighbors.
    // directional deltas
    const int delta[4][2]{{-1, 0},
                          {0,  -1},
                          {1,  0},
                          {0,  1}};
    for (int i=0; i<4; i++){
        int x2 = x+delta[i][0];
        int y2 = y+delta[i][1];
        if (CheckValidCell(x2,y2,grid)){
            int g2 = g+1;
            int h2 = Heuristic(x2,y2,goal[0],goal[1]);
            AddToOpen(x2,y2,g2,h2,open_nodes,grid);
        }

    }

    }


    // TODO: Check that the potential neighbor's x2 and y2 values are on the grid and not closed.

    // TODO: Increment g value, compute h value, and add neighbor to open list.

// } TODO: End the function


/**
 * Implementation of A* search algorithm
 */

std::vector<vector<State>> Search(vector<vector<State>>& grid,
                                  const int (&init)[2],
                                  const int (&goal)[2])
{

    // Create the vector of open nodes.
    vector<vector<int>> open{};

    // TODO: Initialize the starting node.
    int x = init[0];
    int y = init[1];
    int g = 0;
    int h = Heuristic(x, y, goal[0], goal[1]);

    // TODO: Use AddToOpen to add the starting node to the open vector.

    AddToOpen(x, y, g, h, open, grid);


    // TODO: while open vector is non empty {
    while (open.size() > 0)
    {
        // TODO: Sort the open list using CellSort, and get the current node.

        CellSort(&open);
        vector<int> current_node{};
        current_node = open.back();
        open.pop_back();
        // TODO: Get the x and y values from the current node,
        // and set grid[x][y] to kPath.

        int x = current_node[0];
        int y = current_node[1];
        grid[x][y] = State::kPath;
        // TODO: Check if you've reached the goal. If so, return grid.
        if (x == goal[0] && y == goal[1])
        {
            grid[init[0]][init[1]] = State::kStart;
            grid[goal[0]][goal[1]] = State::kFinish;
            return grid;
        }
        else
        {
            ExpandNeighbors(current_node,open,grid,goal);
        }
        // If we're not done, expand search to current node's neighbors. This step will be completed in a later quiz.
        // ExpandNeighbors

        // TODO: End while loop

    }

    cout << "Path found!";

//    std::vector<vector<State>> empty;
    return grid;

}





// TODO: Write the Search function stub here.


string CellString(State cell)
{
    switch(cell) {
        case State::kObstacle: return "⛰️   ";
        case State::kPath: return "🚗   ";
        case State::kStart: return "🚦   ";
        case State::kFinish: return "🏁   ";
        default: return "0   ";
    }
}


void PrintBoard(const vector<vector<State>> board) {
    for (int i = 0; i < board.size(); i++) {
        for (int j = 0; j < board[i].size(); j++) {
            cout << CellString(board[i][j]);
        }
        cout << "\n";
    }
}


int main() {
    // TODO: Declare "init" and "goal" arrays with values {0, 0} and {4, 5} respectively.
    int init[2] = {0, 0};
    int goal[2] = {4, 5};

    auto board = ReadBoardFile("1.board");
    // TODO: Call Search with "board", "init", and "goal". Store the results in the variable "solution".

    auto solution = Search(board,init,goal);

    // TODO: Change the following line to pass "solution" to PrintBoard.
    PrintBoard(solution);
}