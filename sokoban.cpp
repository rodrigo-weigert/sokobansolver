#include <bits/stdc++.h>

/* Macros para converter uma posição na fase entre dois
sistemas de coordenadas um mono e um bidimensonal
O uso da coordenada monodimensional serve para
economizar memória e facilitar certas implementações */
#define get_pos(x, y) (x * cols + y)
#define get_x(pos) (pos / cols)
#define get_y(pos) (pos % cols)


#define MAX_BOXES_PER_LEVEL 20
#define MAX_LEVEL_SIZE 256

/* Caracteres utilizados na representação do nível */
#define PLAYER '@'
#define GOAL '.'
#define BOX '$'
#define BOX_ON_GOAL '*'
#define PLAYER_ON_GOAL '+'
#define WALL '#'

using namespace std;

const short oo = 0x3f3f;										// Infinito
const short dir[4][2] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};		// Vetores das direções do movimento do jogador (direção 0: cima, direção 1: direita...)
const char dirname[4] = {'U', 'R', 'D', 'L'};					// Caracteres correspondendes às direções dos vetores acima

short goals[MAX_BOXES_PER_LEVEL];								// Vetor com as coordenadas (monodimensionais) dos objetivos
char grid[MAX_LEVEL_SIZE][MAX_LEVEL_SIZE+2];					// Representação do nível/tabuleiro/fase fonrecida na entrada
int rows, cols;													// Dimensões do nível
int	nboxes;														// Número de caixas (e de objetivos) do nível


/* A struct state representa um estado do jogo. Este é totalmente determinado pela posição do jogador e de cada caixa */
struct state
{
	short px, py;						//O uso de short permite economia significativa de memória, pois milhões de estados são armazenados
	short boxes[MAX_BOXES_PER_LEVEL];
};

/* Este é o comparador de estados (dados dois estados a e b,
retorna true se a é considerado "menor" que b).
Isto é utilizado para que estados possam ser inseridos em
estruturas de árvores binárias de busca da STL do C++ (ex: std::map).
Tais estruturas são utilizadas pelas funções de busca
(ex: para armazenar estados visitados e seus antecessores na BFS) */
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

//É um nó da fila de prioridade a ser utilizada pelo A*
struct anode
{
	short cost;		// Custo do caminho até agora
	short est;		// Estimativa de custo até o objetivo
	state s;		// Estado associado
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

/* Dado um estado s e uma posição (x,y), retorna true se e
somente se o estado s contém uma caixa na posição (x,y) */
bool has_box(state& s, short x, short y)
{
	for (int i = 0; i < nboxes; i++)
	{
		if (get_x(s.boxes[i]) == x and get_y(s.boxes[i]) == y)
			return true;
	}
	return false;
}

/* Uma posição (x, y) é dita "válida" se ela não é uma parede e se está dentro dos limites do nível */
bool valid(short x, short y)
{
	if (x >= 0 and x < rows and y >= 0 and y < cols)
		return grid[x][y] != WALL;
	else
		return false;
}

/* Dado um estado s e uma direção d, retorna true se e somente
se é possível o jogador realizar um movimento na direção d.
Nesse caso, a função também altera s para representar o estado em que
se chega ao realizar o movimento, movimentando uma caixa, se necessário.
Se não for possível realizar o movimento, retorna false e não altera s.
Um jogador numa posição (x,y) pode se mover na direção (dx, dy) se e somente se
(x + dx, y + dy) é uma posição válida e uma das seguintes condições é verdadeira:
 			- (x + dx, y + dy) não contém uma caixa
 			- (x + dx, y + dy) contém uma caixa e (x + 2*dx, y + 2*dy) é uma posição válida que não contém uma caixa */
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

/* Dado um estado s, verifica se esse estado representa um nível finalizado.
 	 Um nível está finalizado se e somente se todas as caixas ocupam posições objetivo.*/
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
/* Determina a igualdade entre estados. Dois estados são iguais
 se os valores de todos os seus componentes são iguais.
Esta função é utilizada para recuperar o caminho percorrido pelas
funções de busca e portanto o passo a passo da solução do nível. */
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

/* BFS que busca pela solução do nível. Ao encontrar, retorna uma
string que representa a sequencia de movimentos de tal solução */
void bfs(state& initial)
{
	map<state, state, state_cmp> prev;			//Associa a cada estado s o estado que levou a BFS a s.
	queue<state> q;								//Fila de estados a percorrer
	state end; 									//Armazenará o estado final atigindo
	clock_t t = clock();						//Para medição de tempo
	int expl_count = 0;							//Contador de estados explorados

	prev[initial] = initial;
	q.push(initial);
	while (!q.empty())
	{
		state cur = q.front();
		expl_count++;
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
	t = clock() - t;

	/* Recuperação do caminho percorrido */
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
	printf("Search concluded after %lf s, %d states explored.\n", 1.0*t / CLOCKS_PER_SEC, expl_count);
	printf("Solution: %s (%d moves)\n", path.c_str(), (int)path.size());
}

// Função de estimação do custo até a solução a partir de um estado s.
// Soma das menores distâncias (número de movimentos) entre cada caixa e um objetivo qualquer.
// A soma é calculada com uma BFS de múltiplos vértices iniciais
short estimate(state& s)
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

/*Realiza a busca das solucoes utilizando a heuristica A* e retorna a string com a solucao.
Muito similiar a BFS, porem utiliza a estrutura da fila de prioridades ordenada usando uma heurística,
de forma a tentar visitar estados numa ordem que leve à solução mais rapidamente */
void astar(state& initial)
{
	map<state, state, state_cmp> prev; 								//Associa a cada estado s o estado que levou o A* a explorar s.
	priority_queue<anode, vector<anode>, anode_cmp> q;				//Fila de prioridades (conterá os próximos estados a explorar)
	map<state, short, state_cmp> dist;								//Estrutura que armazena as distancias calculadas para os estados visitados
	state end;														//Armazenará o estado final
	anode initial_a;												//Envelope para o estado inicial para ser inserido na fila de prioridade
	clock_t t = clock();											//Para medição de tempo
	int expl_count = 0;												//Contador de estados explorados

	initial_a.cost = 0;
	initial_a.est = estimate(initial);
	initial_a.s = initial;

	prev[initial] = initial;
	dist[initial] = 0;
	q.push(initial_a);
	while (!q.empty())
	{
		anode cur = q.top();
		expl_count++;
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
				next_a.est = estimate(next);
				if (next_a.est < oo)
				{
					prev[next] = cur.s;
					q.push(next_a);
				}
			}
		}
	}
	t = clock() - t;

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
	printf("Heuristic search concluded after %lf s, %d states explored.\n", 1.0*t / CLOCKS_PER_SEC, expl_count);
	printf("Solution: %s (%d moves)\n", path.c_str(), (int)path.size());

}

int main(int argc, char* argv[])
{
	state initial;
	int goalcount = 0;
	bool heuristic = false;
	FILE* input_file;

	if (argc == 1)
	{
		printf("usage:\t%s levelfile [--heuristic]\n", argv[0]);
		return 0;
	}

	input_file = fopen(argv[1], "r");
	if (input_file == NULL)
	{
		printf("Error: file not found.\n");
		return 0;
	}

	if (argc == 3 and !strcmp(argv[2], "--heuristic"))
		heuristic = true;

	rows = cols = 0;
	while (fgets(grid[rows], MAX_LEVEL_SIZE, input_file) == grid[rows])
	{
		int i = rows++;
		cols = max(cols, (int)strlen(grid[i])-1);	
	}
	
	for (int i = 0; i < rows; i++)
	{
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
	if (heuristic)
		astar(initial);
	else
		bfs(initial);
	close(input_file);
	return 0;
}
