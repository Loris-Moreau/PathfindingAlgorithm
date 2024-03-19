#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>


constexpr int rows = 10;
constexpr int cols = 10;

struct Node
{
    int x, y; // Position of the node
    float g, h, f; // Cost values
    bool obstacle; // Flag to indicate if the node is an obstacle
    bool operator==(const Node& other) const
    {
        return x == other.x && y == other.y;
    }
};

float distance(const Node& node1, const Node& node2)
{
    return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
}

std::vector<Node> astar(const Node& start, const Node& goal, std::vector<std::vector<Node>>& grid)
{
    std::vector<Node> openList, closedList;
    openList.push_back(start);

    while (!openList.empty())
    {
        // Get the node with the lowest f value
        auto currentNode = min_element(openList.begin(), openList.end(), [](const Node& a, const Node& b)
        {
            return a.f < b.f;
        });
        Node current = *currentNode;
        openList.erase(currentNode);

        std::cout << "Expanding node: (" << current.x << ", " << current.y << ")" << '\n';

        closedList.push_back(current);
        
        // Found the goal
        if (current == goal)
        {
            std::vector<Node> path;
            while (!(current == start))
            {
                path.push_back(current);
                // Backtrack to get the path
                auto parent = find_if(closedList.begin(), closedList.end(), [&](const Node& node)
                {
                    return node == current;
                });
                if (parent == closedList.end())
                {
                    std::cerr << "Error: Parent not found for node: (" << current.x << ", " << current.y << ")" << '\n';
                    return {}; // Return empty path
                }
                current = *parent;
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        // Generate children
        for (int dx = 0; dx <= 1; ++dx)
        {
            for (int dy = 0; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0)
                {
                    continue; // Skip the current node
                }
                int newX = current.x + dx;
                int newY = current.y + dy;
                
                if(newX < 0 || newX > rows)
                {
                    std::cout << "error X out of bounds" << '\n';
                    newX = 0;
                    
                    if(newY < 0 || newY > cols)
                    {
                        std::cout << "error Y out of bounds" << '\n';
                        newY = 0;
                    }
                }

                /*// Check if the new coordinates are within the grid boundaries
                if (newX < 0 || newX >= grid.size() || newY < 0 || newY >= grid[0].size())
                {
                    std::cout << "Child node: (" << newX << ", " << newY << ") is out of grid bounds." << '\n';
                    continue;
                }
                */

                Node& child = grid[newX][newY];

                // Check if the child is an obstacle
                if (child.obstacle)
                {
                    std::cout << "Child node: (" << newX << ", " << newY << ") is an obstacle." << '\n';
                    continue;
                }

                // Calculate g, h, and f values
                const float tentative_g = current.g + distance(current, child);
                const float h = distance(child, goal);
                const float f = tentative_g + child.h;

                // Check if the child is already in the closed list
                if (find(closedList.begin(), closedList.end(), child) != closedList.end())
                {
                    std::cout << "Child node: (" << newX << ", " << newY << ") is already in closed list." << '\n';
                    continue;
                }

                // Check if the child is already in the open list
                auto it = find(openList.begin(), openList.end(), child);
                if (it != openList.end())
                {
                    if (tentative_g >= it->g)
                    {
                        std::cout << "Child node: (" << newX << ", " << newY << ") is already in open list with higher g value." << '\n';
                        continue;
                    }
                }

                // Update child values
                child.g = tentative_g;
                child.h = h;
                child.f = f;

                // Add child to open list
                openList.push_back(child);
            }
        }
    }

    // If no path is found, return an empty path
    std::cerr << "Error: No path found!" << '\n';
    return {};
}

// Print the grid with the path
void printGridWithPath(const std::vector<std::vector<Node>>& grid, const std::vector<Node>& path)
{
    for (const auto& i : grid)
    {
        for (const auto& j : i)
        {
            if (std::find(path.begin(), path.end(), j) != path.end())
            {
                std::cout << "* "; // Mark path nodes with *
            }
            else if (j.obstacle)
            {
                std::cout << "X "; // Mark obstacle nodes with X
            }
            else
            {
                std::cout << "E "; // Mark empty nodes with E
            }
        }
        std::cout << '\n';
    }
}

int main()
{
    std::vector<std::vector<Node>> grid(rows, std::vector<Node>(cols));
    
    // Initialize grid with nodes and set obstacles
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            grid[i][j].x = i;
            grid[i][j].y = j;
            grid[i][j].obstacle = false;
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

    constexpr Node start{0, 0, 0, 0, 0, false}; // Set start node
    constexpr Node goal{rows - 1, cols - 1, 0, 0, 0, false};  // Set goal node

    // Call A* algorithm
    const std::vector<Node> path = astar(start, goal, grid);

    // Print the path
    if (!path.empty())
    {
        std::cout << "Path found :" << '\n';
        for (const auto& node : path)
        {
            std::cout << "(" << node.x << ", " << node.y << ") ";
        }
        std::cout << '\n';

        // Print the grid with the path marked
        std::cout << "Grid with path :" << '\n';
        printGridWithPath(grid, path);
    }
    else
    {
        std::cout << "No path found !" << '\n';
    }

    return 0;
}
