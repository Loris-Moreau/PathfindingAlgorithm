#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

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
                current = *parent;
            }
            path.push_back(start);
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
                int newX = current.x + dx;
                int newY = current.y + dy;

                // Check if the new coordinates are within the grid boundaries
                if (newX < 0 || newX >= grid.size() || newY < 0 || newY >= grid[0].size())
                {
                    continue;
                }

                Node& child = grid[newX][newY];

                // Check if the child is an obstacle
                if (child.obstacle)
                {
                    continue;
                }

                // Calculate g, h, and f values
                float tentative_g = current.g + distance(current, child);
                float h = distance(child, goal);
                float f = tentative_g + h;

                // Check if the child is already in the closed list
                if (find(closedList.begin(), closedList.end(), child) != closedList.end())
                {
                    continue;
                }

                // Check if the child is already in the open list
                auto it = find(openList.begin(), openList.end(), child);
                if (it != openList.end())
                {
                    if (tentative_g >= it->g)
                    {
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
    return {};
}

int main()
{
    // Example usage
    int rows = 10;
    int cols = 10;
    std::vector<std::vector<Node>> grid(rows, std::vector<Node>(cols));
    // Initialize grid with nodes and set obstacles
    // For simplicity, assuming all nodes are initially not obstacles

    Node start{0, 0, 0, 0, 0, false}; // Set start node
    Node goal{rows - 1, cols - 1, 0, 0, 0, false};  // Set goal node

    //Obstacles
    Node obstacle1{5, 0, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle2{5, 1, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle3{5, 2, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle4{5, 3, 0, 0, 0, true};  // Set Obstacle node
    
    Node obstacle5{5, 6, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle6{5, 7, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle7{5, 8, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle8{5, 9, 0, 0, 0, true};  // Set Obstacle node
    
    Node obstacle9{7, 3, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle10{7, 4, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle11{7, 5, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle12{7, 6, 0, 0, 0, true};  // Set Obstacle node
    
    Node obstacle13{2, 1, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle14{2, 2, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle15{2, 3, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle16{2, 4, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle17{2, 5, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle18{2, 6, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle19{2, 7, 0, 0, 0, true};  // Set Obstacle node
    Node obstacle20{2, 8, 0, 0, 0, true};  // Set Obstacle node
    
    // Call A* algorithm
    std::vector<Node> path = astar(start, goal, grid);

    // Print the path
    if (!path.empty())
    {
        std::cout << "Path found:" << '\n';
        for (const auto& node : path)
        {
            std::cout << "(" << node.x << "," << node.y << ") ";
        }
        std::cout << '\n';
    } else
    {
        std::cout << "No path found!" << '\n';
    }
    return 0;
}
