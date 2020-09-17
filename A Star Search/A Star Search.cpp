#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm> // for sort

using namespace std;

enum class State { kEmpty, kObstacle, kClosed, kStart, kFinish, kPath}; // define a new type: 'State'

string CellString( State cell ){
    // take a cell's State as input, output its print value
    switch (cell) {
        case State::kObstacle: return "X   ";
        case State::kPath: return "P   ";
        case State::kStart: return "S   ";
        case State::kFinish: return "F   ";
        default: return "0   ";
    }
}

vector<State> ParseLine(string s) {
    // Input: a file's row (str)
    // Return: a vector containing the elements in this row

    istringstream sstream(s);
    char comma;
    int number;
    vector<State> row;
    while (sstream >> number >> comma) { // read two chars at one time
        if (number == 0) { // 0 is empty space, 1 is obstacle
            row.push_back(State::kEmpty);
        }
        else {
            row.push_back(State::kObstacle);
        }
    }
    return row;
}

vector<vector<State>> ReadGridFile(string filePath) {
    // Input: the path of a board file
    // Return:  2D vector contained in the file

    ifstream file(filePath);
    string line;
    vector<vector<State>> grid;

    while (getline(file, line)) {
        grid.push_back(ParseLine(line));
    }
    return grid;
}


void PrintGrid(const vector<vector<State>> grid) {
    // print the board
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[i].size(); j++) {
            cout << CellString(grid[i][j]);
        }
        cout << "\n";
    }
}



int Heuristic(int x1, int y1, int x2, int y2) {
    // Inputs: two xy coordinates
    // Return: the Manhattan Distance from one coordinate to the other |x2-x1|+|y2-y1|

    return abs(x2 - x1) + abs(y2 - y1);
}

bool CompareScore(const vector<int> node1, const vector<int> node2) {
    // Inputs: two nodes of type vector<int>
    // Return: true if f-value of first argument > the second, false otherwise

    int f1 = node1[2] + node1[3]; // f = g + h
    int f2 = node2[2] + node2[3];

    return (f1 > f2);
}

bool CheckValidCell(int x, int y, vector<vector<State>> &grid) {
    // Inputs: x y coordinates, the grid
    // Return: true if cell is on the grid and is empty, false otherwise

    bool isInGrid = (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size());
    if (isInGrid) {
        return grid[x][y] == State::kEmpty;
    }
    return false;
}



void AddToFrontier(int x, int y, int g, int h, vector<vector<int>>& frontier, vector<vector<State>>& grid) {

    vector<int> node = { x, y, g, h };
    /* a node contains 4 values:
    1. x coordinate
    2. y coordinate,
    3. g value(or cost) that has accumulated up to that cell
    4. h value for the cell, given by the Heuristic function
    */

    frontier.push_back(node); // frontier contains nodes that are to be explored

    // mark the node as closed, so that algorithm does not search nodes that are already in frontier
    // but instead search the neighbors of frontier nodes
    grid[x][y] = State::kClosed;

}

void SortFrontier(vector<vector<int>>* frontier) {
    // Sort a vector in descending order 

    sort(frontier->begin(), frontier->end(), CompareScore);
}

void ExpandNeighbors(vector<int>& node, vector<vector<int>>& frontier, vector<vector<State>>& grid, int goal[2]) {
    int x = node[0];
    int y = node[1];
    int g = node[2];

    vector<int> neighbor_x = { x - 1,       x,       x + 1,       x };  //left, top, right, bot
    vector<int> neighbor_y = { y    ,   y + 1,       y,       y - 1 };
    int neighbor_h;

    for (int i = 0; i <= 3; i++) {
        if (CheckValidCell(neighbor_x[i], neighbor_y[i], grid)) {
            neighbor_h = Heuristic(neighbor_x[i], neighbor_y[i], goal[0], goal[1]);
            AddToFrontier(neighbor_x[i], neighbor_y[i], g + 1, neighbor_h, frontier, grid); // g(cost) of each step is 1
        }
    }

}

vector<vector<State>> Search(vector<vector<State>> grid, int start[2], int goal[2]) {
    // Inputs: a grid, a start array (coordinates), and a goal array (coordinates)
    // Return: grid with a path from the start to the goal

    vector<vector<int>> frontier{};
    int x = start[0];
    int y = start[1];
    int g = 0;
    int h = Heuristic(x, y, goal[0], goal[1]);

    AddToFrontier(x, y, g, h, frontier, grid); // add the start node to frontier

    while (frontier.size() > 0) {
        SortFrontier(&frontier);
        vector<int> curNode = frontier.back(); //last has least f score
        frontier.pop_back(); // remove from frontier, already marked closed earlier
        x = curNode[0];
        y = curNode[1];
        grid[x][y] = State::kPath;

        if (x == goal[0] && y == goal[1]) { // found goal, return
            grid[start[0]][start[1]] = State::kStart;
            grid[goal[0]][goal[1]] = State::kFinish;
            return grid;
        }
        else {
            ExpandNeighbors(curNode, frontier, grid, goal);
        }
    }
    cout << "No path found!" << "\n"; // loop ends without return
    return vector<vector<State>>{};
}

int main() {
    vector<vector<State>> grid = ReadGridFile("b.board");
    int start[2]{ 0, 0 };
    int goal[2]{ 4, 5 };

    auto solution = Search(grid, start, goal);

    PrintGrid(solution);
}
