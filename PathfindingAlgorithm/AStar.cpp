#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

constexpr int rows = 10;
constexpr int cols = 10;

enum Terrain
{
    Normal,
    Challenging,
    Dificult,
    Obstacle
};

struct Node
{
    int x, y; // Position of the node
    float g, h, f; // Cost values
    bool obstacle; // Flag to indicate if the node is an obstacle
    Node* parent; // Pointer to the parent node
    bool operator==(const Node& other) const
    {
        return x == other.x && y == other.y;
    }
    Terrain terrain;
};

float distance(const Node& node1, const Node& node2)
{
    return sqrt(std::pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
}

std::vector<Node*> astar(Node* start, Node* goal, std::vector<std::vector<Node>>& grid)
{
    start->terrain = Terrain::Normal;
    
    std::vector<Node*> openList, closedList;
    start->g = 0;
    start->h = distance(*start, *goal);
    start->f = start->g + start->h;
    openList.push_back(start);

    while (!openList.empty())
    {
        // Get the node with the lowest f value
        auto currentNode = min_element(openList.begin(), openList.end(), [](const Node* a, const Node* b)
        {
            return a->f < b->f;
        });
        Node* current = *currentNode;
        openList.erase(currentNode);

        closedList.push_back(current);

        // Found the goal
        if (*current == *goal)
        {
            std::vector<Node*> path;
            while (current != nullptr)
            {
                path.push_back(current);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // Generate children
        for (int dx = -1; dx <= 1; ++dx)
            {
            for (int dy = -1; dy <= 1; ++dy)
                {
                if (dx == 0 && dy == 0)
                {
                    continue; // Skip the current node
                }
                int newX = current->x + dx;
                int newY = current->y + dy;

                if (newX < 0 || newX >= rows || newY < 0 || newY >= cols)
                {
                    continue; // Out of grid bounds
                }

                Node& child = grid[newX][newY];

                // Check if the child is an obstacle
                if (child.obstacle)
                {
                    continue;
                }

                // Child is on the closedList
                if (find(closedList.begin(), closedList.end(), &child) != closedList.end())
                {
                    continue;
                }

                float tentative_g = current->g + distance(*current, child);
                bool isNewPath = false;

                // Check if the child is already in the open list
                auto it = find(openList.begin(), openList.end(), &child);
                if (it == openList.end())
                {
                    isNewPath = true;
                    child.h = distance(child, *goal);
                    openList.push_back(&child);
                }
                else if (tentative_g < child.g)
                {
                    isNewPath = true;
                }

                if (isNewPath)
                {
                    child.parent = current;
                    child.g = tentative_g;
                    child.f = child.g + child.h;
                }
            }
        }
    }

    // If no path is found, return an empty path
    return {};
}

void printGridWithPath(const std::vector<std::vector<Node>>& grid, const std::vector<Node*>& path)
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            const Node& node = grid[i][j];
            Node nodal = grid[i][j];
            if (find(path.begin(), path.end(), &node) != path.end())
            {
                std::cout << "* "; // Mark path nodes with *
                nodal.terrain = Normal;
            }
            else if (node.obstacle)
            {
                std::cout << "X "; // Mark obstacle nodes with X
                nodal.terrain = Obstacle;
            }
            else
            {
                std::cout << "E "; // Mark empty nodes with E
                nodal.terrain = Normal;
            }
        }
        std::cout << '\n';
    }
}

int main() {
    std::vector<std::vector<Node>> grid(rows, std::vector<Node>(cols));

    // Initialize grid with nodes and set obstacles
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            grid[i][j].x = i;
            grid[i][j].y = j;
            grid[i][j].obstacle = false;
            grid[i][j].parent = nullptr;
        }
    }

    // Set obstacles
    grid[5][0].obstacle = true;
    grid[5][1].obstacle = true;
    grid[5][2].obstacle = true;
    grid[5][3].obstacle = true;
    grid[5][6].obstacle = true;
    grid[5][7].obstacle = true;
    grid[5][8].obstacle = true;
    grid[5][9].obstacle = true;
    grid[7][3].obstacle = true;
    grid[7][4].obstacle = true;
    grid[7][5].obstacle = true;
    grid[7][6].obstacle = true;
    grid[2][1].obstacle = true;
    grid[2][2].obstacle = true;
    grid[2][3].obstacle = true;
    grid[2][4].obstacle = true;
    grid[2][5].obstacle = true;
    grid[2][6].obstacle = true;
    grid[2][7].obstacle = true;
    grid[2][8].obstacle = true;

    Node* start = grid[0].data(); // Set start node
    Node* goal = &grid[rows - 1][cols - 1];  // Set goal node

    // Call A* algorithm
    std::vector<Node*> path = astar(start, goal, grid);

    // Print the path
    if (!path.empty())
    {
        std::cout << "Path found :\n";
        for (const auto& node : path)
        {
            std::cout << "(" << node->x << ", " << node->y << ") ";
        }
        std::cout << '\n';

        // Print the grid with the path marked
        std::cout << "Grid with path :\n";
        printGridWithPath(grid, path);
    }
    else
    {
        std::cout << "No path found !\n";
    }

    return 0;
}
