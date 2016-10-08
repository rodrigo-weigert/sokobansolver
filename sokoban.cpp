#include <bits/stdc++.h>
#define get_pos(x, y) (x * cols + y)
#define get_x(pos) (pos / cols)
#define get_y(pos) (pos % cols)
#define MAX_BOXES_PER_LEVEL 20
#define MAX_LEVEL_SIZE 256
#define PLAYER '@'
#define GOAL '.'
#define BOX '$'
#define BOX_ON_GOAL '*'
#define PLAYER_ON_GOAL '+'
#define WALL '#'

using namespace std;

const short oo = 0x3f3f;
const short dir[4][2] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
const char dirname[4] = {'U', 'R', 'D', 'L'};
short goals[MAX_BOXES_PER_LEVEL];

char grid[MAX_LEVEL_SIZE][MAX_LEVEL_SIZE+2];
int rows, cols;
int	nboxes;

struct state
{
	short px, py;
	short boxes[MAX_BOXES_PER_LEVEL];
};

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

struct anode		//É um nó da fila de prioridade a ser utilizada pelo A*
{
	short cost;		// custo do caminho até agora
	short est;		// estimativa de custo até o objetivo
	state s;		// estado associado
};

struct anode_cmp
{
	bool operator() (const anode& a, const anode& b) const
	{		
		/* Um anode (nó que envelopa um estado) é considerado 'menor'
		que outro se o valor de sua função heurística
		(custo do passado + estimativa do futuro) tem valor maior
		que a do outro. A fila de prioridade do A* colocará em seu topo
		os  maiores estados (envelopados em anodes). */
		return (a.cost + a.est) > (b.cost + b.est);
	}
};

bool has_box(state& s, short x, short y)
{
	for (int i = 0; i < nboxes; i++)
	{
		if (get_x(s.boxes[i]) == x and get_y(s.boxes[i]) == y)
			return true;
	}
	return false;
}

bool valid(short x, short y)
{
	if (x >= 0 and x < rows and y >= 0 and y < cols)
		return grid[x][y] != WALL;
	else
		return false;
}

bool move(state& s, int d)
{
	short x = s.px + dir[d][0];
	short y = s.py + dir[d][1];

	if (valid(x, y))
	{
		if (has_box(s, x, y))
		{
			short bx = x + dir[d][0];
			short by = y + dir[d][1];

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
		short x = get_x(s.boxes[i]);
		short y = get_y(s.boxes[i]);
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
	map<state, state, state_cmp> prev;
	queue<state> q;
	state end;

	prev[initial] = initial;
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
			if(move(next, d) and !prev.count(next))
			{
				q.push(next);
				prev[next] = cur;
			}
		}
	}
	string path;
	state cur = end;
	while (!equal(cur, initial))
	{
		state prv = prev[cur];
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

short estimate(state& s) //Soma das menores distâncias (de Manhattan) entre cada caixa e um objetivo.
{
	short est = 0;
	for (int i = 0; i < nboxes; i++)
	{
		short mindist = oo;
		for (int j = 0; j < nboxes; j++)
		{
			mindist = min(mindist, (short)(abs(get_x(s.boxes[i]) - get_x(goals[j])) + abs(get_y(s.boxes[i]) - get_y(goals[j]))));
		}
		est += mindist;
	}
	return est;
}

short estimate2(state& s) //Soma das menores distancias (em numero real de movimentos) entre cada caixa e um objetivo
{
	queue<short> q;
	short dist[MAX_LEVEL_SIZE][MAX_LEVEL_SIZE];

	memset(dist, -1, sizeof(dist));

	for (int i = 0; i < nboxes; i++)
	{
		dist[get_x(goals[i])][get_y(goals[i])] = 0;
		q.push(goals[i]);
	}

	while (!q.empty())
	{
		short x = get_x(q.front());
		short y = get_y(q.front());
		q.pop();

		for (int d = 0; d < 4; d++)
		{
			short next_x = x + dir[d][0];
			short next_y = y + dir[d][1];

			if (valid(next_x, next_y) and valid(next_x + dir[d][0], next_y + dir[d][1]) and dist[next_x][next_y] == -1)
			{
				dist[next_x][next_y] = 1 + dist[x][y];
				q.push(get_pos(next_x, next_y));
			}
		}
	}
	short est = 0;
	for (int i = 0; i < nboxes; i++)
	{
		if (dist[get_x(s.boxes[i])][get_y(s.boxes[i])] == -1)
			return oo;
		est += dist[get_x(s.boxes[i])][get_y(s.boxes[i])];
	}
	return est;
}

string astar(state& initial)
{
	map<state, state, state_cmp> prev;
	priority_queue<anode, vector<anode>, anode_cmp> q;
	map<state, short, state_cmp> dist;
	state end;
	anode initial_a;

	initial_a.cost = 0;
	initial_a.est = estimate(initial);
	initial_a.s = initial;

	prev[initial] = initial;
	dist[initial] = 0;
	q.push(initial_a);
	while (!q.empty())
	{
		anode cur = q.top();
		q.pop();

		if (cur.cost > dist[cur.s])
			continue;

		if (cleared(cur.s))
		{
			end = cur.s;
			break;
		}
		for (int d = 0; d < 4; d++)
		{
			state next = cur.s;
			if (move(next, d) and (!dist.count(next) or dist[next] > cur.cost + 1))
			{
				anode next_a;
				next_a.s = next;
				next_a.cost = dist[next] = cur.cost + 1;
				next_a.est = estimate2(next);
				if (next_a.est < oo)
				{
					prev[next] = cur.s;
					q.push(next_a);
				}
			}
		}
	}
	/* A parte abaixo (reconstrução do caminho) é identica a da BFS */
	string path;
	state cur = end;
	while (!equal(cur, initial))
	{
		state prv = prev[cur];
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

	int goalcount = 0;
	for (int i = 0; i < rows; i++)
	{
		fgets(grid[i], MAX_LEVEL_SIZE, stdin);
		for (int j = 0; j < cols; j++)
		{
			if (grid[i][j] == PLAYER or grid[i][j] == PLAYER_ON_GOAL)
			{
				initial.px = (short)i;
				initial.py = (short)j;
			}
			if (grid[i][j] == BOX or grid[i][j] == BOX_ON_GOAL)
				initial.boxes[nboxes++] = get_pos(i, j);
			if (grid[i][j] == GOAL or grid[i][j] == BOX_ON_GOAL or grid[i][j] == PLAYER_ON_GOAL)
				goals[goalcount++] = get_pos(i, j);
		}
	}
	string bfs_ans, astar_ans;
	double bfs_time, astar_time;
	int s, t;

	s = clock();
	astar_ans = astar(initial);
	t = clock();
	astar_time = (1.0*(t-s)) / CLOCKS_PER_SEC;
	printf("A* (%lf s):\t%s\n", astar_time, astar_ans.c_str());

	s = clock();
	bfs_ans = bfs(initial);
	t = clock();
	bfs_time = (1.0*(t-s)) / CLOCKS_PER_SEC;
	printf("BFS: (%lf s):\t%s\n", bfs_time, bfs_ans.c_str());

	return 0;
}
