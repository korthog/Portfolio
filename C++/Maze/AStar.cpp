
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

#include "Bitmap.cpp"
#include "InterleaveBits.h"

enum class Direction : unsigned char
{
	North = 0,
	NorthEast = 1,
	East = 2,
	SouthEast = 3,
	South = 4,
	SouthWest = 5,
	West = 6,
	NorthWest = 7
};

struct Node
{
	int x;
	int y;
	Direction dir;

	constexpr inline bool operator==(const Node& other) const = default;

	struct Hash
	{
		size_t operator()(const Node& n) const
		{
			return 8 * interleaveBits(n.x, n.y) + static_cast<unsigned char>(n.dir);
		}
	};
};

struct Path
{
	Node node = Node();
	float cost = std::numeric_limits<float>::infinity();

	constexpr inline bool operator<(const Path& other) const { return cost > other.cost; }
};

class Neighbors
{
	static constexpr int size = 3;
	static constexpr float straightCost = 1.0f;
	static constexpr float diagCost = 1.415f;
	static constexpr float turnCost = 0.1f;

	Path neighbors[size];

public:
	Neighbors(const Node& current)
	{
		switch (current.dir)
		{
		case Direction::North:
			neighbors[0] = Path(Node(current.x, current.y + 1, current.dir), straightCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::NorthEast), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::NorthWest), turnCost);
			break;
		case Direction::NorthEast:
			neighbors[0] = Path(Node(current.x + 1, current.y + 1, current.dir), diagCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::East), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::North), turnCost);
			break;
		case Direction::East:
			neighbors[0] = Path(Node(current.x + 1, current.y, current.dir), straightCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::SouthEast), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::NorthEast), turnCost);
			break;
		case Direction::SouthEast:
			neighbors[0] = Path(Node(current.x + 1, current.y - 1, current.dir), diagCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::South), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::East), turnCost);
			break;
		case Direction::South:
			neighbors[0] = Path(Node(current.x, current.y - 1, current.dir), straightCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::SouthWest), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::SouthEast), turnCost);
			break;
		case Direction::SouthWest:
			neighbors[0] = Path(Node(current.x - 1, current.y - 1, current.dir), diagCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::West), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::South), turnCost);
			break;
		case Direction::West:
			neighbors[0] = Path(Node(current.x - 1, current.y, current.dir), straightCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::NorthWest), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::SouthWest), turnCost);
			break;
		case Direction::NorthWest:
			neighbors[0] = Path(Node(current.x - 1, current.y + 1, current.dir), diagCost);
			neighbors[1] = Path(Node(current.x, current.y, Direction::North), turnCost);
			neighbors[2] = Path(Node(current.x, current.y, Direction::West), turnCost);
			break;
		}
	}

	Path* begin() { return neighbors; }
	Path* end() { return neighbors + size; }
};

class OccupancyMap
{
	Bitmap bmp;

public:
	OccupancyMap(const Bitmap& map)
	{
		bmp = map;
	}

	bool operator()(const Node& n) const
	{
		return n.x < 0 || n.x >= bmp.Width
			|| n.y < 0 || n.y >= bmp.Height
			|| bmp.getPixel(n.y, n.x).B >= 128;
	}
};

inline float heuristic(const Node& current, const Node& goal)
{
	const float dx = static_cast<float>(current.x - goal.x);
	const float dy = static_cast<float>(current.y - goal.y);
	return sqrt(dx * dx + dy * dy);
}

auto astar(const OccupancyMap& occupied, const Node& start, const Node& goal)
{
	std::priority_queue<Path> queue;
	std::unordered_map<Node, Path, Node::Hash> costMap;

	if (!occupied(start) && !occupied(goal))
	{
		auto startPath = Path(start, 0);
		costMap[start] = startPath;
		queue.push(startPath);
	}

	while (!queue.empty())
	{
		Path current = queue.top();
		queue.pop();

		if (current.node == goal)
			break;

		float currentCost = costMap[current.node].cost;

		for (auto neighbor : Neighbors(current.node))
			if (!occupied(neighbor.node) && costMap[neighbor.node].cost > currentCost + neighbor.cost)
			{
				costMap[neighbor.node] = Path(current.node, currentCost + neighbor.cost);
				queue.push(Path(neighbor.node, currentCost + neighbor.cost + heuristic(neighbor.node, goal)));
			}
	}

	return costMap;
}

std::vector<Node> getRoute(const std::unordered_map<Node, Path, Node::Hash>& map, const Node& goal)
{
	auto route = std::vector<Node>();

	Node current, next = goal;

	do
	{
		current = next;
		route.push_back(current);
		next = map.at(current).node;
	} while (next != current);

	return route;
}

void drawRoute(const std::vector<Node>& route, Bitmap& bmp)
{
	for (auto n : route)
		bmp.setPixel(n.y, n.x, Color::Red());
}

int main()
{
	auto bmp = Bitmap();
	bmp.readFile(R"(C:\Users\korth\Documents\Code\Cpp\Astar\AStar2.bmp)");

	auto occupied = OccupancyMap(bmp);
	auto start = Node(1, 1, Direction::North);
	auto goal = Node(100, 75, Direction::South);

	auto costMap = astar(occupied, start, goal);
	auto route = getRoute(costMap, goal);

	drawRoute(route, bmp);
	bmp.writeFile(R"(C:\Users\korth\Documents\Code\Cpp\Astar\AStar2Output.bmp)");

	std::cout << costMap[goal].cost << std::endl;
}
