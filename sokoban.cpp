#include <bits/stdc++.h>
#define get_pos(x, y) (x * cols + y)
#define get_x(pos) pos / cols
#define get_y(pos) pos % cols
#define MAX_BOXES_PER_LEVEL 20
#define MAX_LEVEL_SIZE 512
#define PLAYER '@'
#define GOAL '.'
#define BOX '$'
#define BOX_ON_GOAL '*'
#define PLAYER_ON_GOAL '+'
#define WALL '#'

using namespace std;


const int dir[4][2] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
const char dirname[4] = {'U', 'R', 'D', 'L'};

char grid[MAX_LEVEL_SIZE][MAX_LEVEL_SIZE+2];
int rows, cols;
int	nboxes;

struct state
{
	int px, py;
	int boxes[MAX_BOXES_PER_LEVEL];
};

bool has_box(state& s, int x, int y)
{
	for (int i = 0; i < nboxes; i++)
	{
		if (get_x(s.boxes[i]) == x and get_y(s.boxes[i]) == y)
			return true;
	}
	return false;
}

struct state_cmp
{
	bool operator() (const state& a, const state& b) const
	{
		if (a.px != b.px)
			return a.px < b.px;
		
		if (a.py != b.py)
			return a.py < b.py;

		for (int i = 0; i < nboxes; i++)
		{	
			if (a.boxes[i] != b.boxes[i])
				return a.boxes[i] < b.boxes[i];
		}
		return false;
	}
};

bool valid(int x, int y)
{
	if (x >= 0 and x < rows and y >= 0 < cols)
		return grid[x][y] != WALL;
	else
		return false;
}

bool move(state& s, int d)
{
	int x = s.px + dir[d][0];
	int y = s.py + dir[d][1];

	if (valid(x, y))
	{
		if (has_box(s, x, y))
		{
			int bx = x + dir[d][0];
			int by = y + dir[d][1];

			if (valid(bx, by) and !has_box(s, bx, by))
			{
				s.px = x;
				s.py = y;
				for (int i = 0; i < nboxes; i++)
				{
					if (get_x(s.boxes[i]) == x and get_y(s.boxes[i]) == y)
					{
						s.boxes[i] = get_pos(bx, by);
						break;
					}
				}
				return true;
			}
			else
				return false;
		}
		else
		{
			s.px = x;
			s.py = y;
			return true;
		}
	}
	else
		return false;
}

bool cleared(state& s)
{
	for (int i = 0; i < nboxes; i++)
	{
		int x = get_x(s.boxes[i]);
		int y = get_y(s.boxes[i]);
		if (grid[x][y] != GOAL and grid[x][y] != BOX_ON_GOAL and grid[x][y] != PLAYER_ON_GOAL)
			return false;
	}
	return true;
}

bool equal(const state& a, const state& b)
{
	if (a.px != b.px)
		return false;
	if (a.py != b.py)
		return false;
	for (int i = 0; i < nboxes; i++)
	{
		if (a.boxes[i] != b.boxes[i])
			return false;
	}
	return true;
}

string bfs(state& initial)
{
	map<state, state, state_cmp> m;
	queue<state> q;
	state end;

	q.push(initial);
	while (!q.empty())
	{
		state cur = q.front();
		q.pop();
		if (cleared(cur))
		{
			end = cur;
			break;
		}
		for (int d = 0; d < 4; d++)
		{
			state next = cur;
			if(move(next, d) and !m.count(next))
			{
				q.push(next);
				m[next] = cur;
			}
		}
	}
	string path;
	state cur = end;
	while (!equal(cur, initial))
	{
		state prv = m[cur];
		for (int d = 0; d < 4; d++)
		{
			if (prv.px + dir[d][0] == cur.px and prv.py + dir[d][1] == cur.py)
			{
				path.push_back(dirname[d]);
				break;
			}
		}
		cur = prv;
	}
	reverse(path.begin(), path.end());
	return path;
}

int main()
{
	state initial;
	scanf("%d %d", &rows, &cols);
	getchar();

	for (int i = 0; i < rows; i++)
	{
		fgets(grid[i], MAX_LEVEL_SIZE, stdin);
		for (int j = 0; j < cols; j++)
		{
			if (grid[i][j] == PLAYER or grid[i][j] == PLAYER_ON_GOAL)
			{
				initial.px = i;
				initial.py = j;
			}
			else if (grid[i][j] == BOX)
				initial.boxes[nboxes++] = get_pos(i, j);
		}
	}
	printf("%s\n", bfs(initial).c_str());
	return 0;
}
