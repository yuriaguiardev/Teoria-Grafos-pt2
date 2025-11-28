

"""
Projeto: Caminhamentos e buscas em grafos e arvores
Contem:
 - Implementacao BFS (busca em largura)
 - Implementacao DFS (busca em profundidade, recursiva e iterativa)
 - Implementacao Dijkstra (caminho minimo para pesos nao-negativos)
 - Implementacao Bellman-Ford (caminho minimo com possivel peso negativo, detecta ciclo negativo)
 - Grafos de exemplo com pelo menos 16 vertices
 - Exemplos/demos que mostram problemas simulados:
    * BFS: busca de menor numero de arestas entre dois nos (ex.: em rede de contatos)
    * DFS: deteccao de componentes e percurso em profundidade (ex.: exploracao de labirinto/arvore)
    * Dijkstra: menor caminho em mapa com pesos positivos (ex.: malha de ruas com distancias)
    * Bellman-Ford: menor caminho com possibilidade de pesos negativos (ex.: ajuste de custos, fluxo)
"""

from collections import deque, defaultdict
import heapq
import math
import sys

# ----------------------------
# Estruturas basicas de grafo
# ----------------------------
class Graph:
    """
    Grafo orientado ou nao orientado com pesos opcionais.
    Representacao: lista de adjacencia.
    Cada entrada: adj[u] = list of (v, weight) para grafos ponderados
    Se peso = None, considera-se aresta nao ponderada (peso implicito 1).
    """
    def __init__(self, num_vertices, directed=False):
        self.n = num_vertices
        self.directed = directed
        # usar dicionario: chave=vertice (0..n-1), valor=list de (v, w)
        self.adj = {i: [] for i in range(self.n)}

    def add_edge(self, u, v, weight=None):
        """Adiciona aresta u->v (e v->u se não for orientado)."""
        self.adj[u].append((v, weight))
        if not self.directed:
            self.adj[v].append((u, weight))

    def neighbors(self, u):
        """Retorna lista de vizinhos (v, weight)."""
        return self.adj[u]

    def __str__(self):
        s = []
        for u in range(self.n):
            s.append(f"{u}: {self.adj[u]}")
        return "\n".join(s)


# ----------------------------
# BFS - Busca em largura
# ----------------------------
def bfs(graph: Graph, start):
    """
    BFS que retorna:
     - distancia_em_arestas: lista com numero de arestas do start ate cada vertice (None se nao atingivel)
     - predecessor: lista com predecessor para reconstruir caminho
    Uso em problema: encontra caminho com menor numero de arestas entre dois nos (unweighted).
    """
    n = graph.n
    dist = [None] * n
    pred = [None] * n
    q = deque()
    dist[start] = 0
    q.append(start)

    while q:
        u = q.popleft()
        for v, w in graph.neighbors(u):
            # para BFS usamos apenas conexoes (peso ignorado), assumimos grafo nao direcionado ou direcionado conforme criado
            if dist[v] is None:
                dist[v] = dist[u] + 1
                pred[v] = u
                q.append(v)
    return dist, pred

def reconstruct_path(pred, start, goal):
    """Reconstrói caminho a partir da lista de predecessores (se existir)."""
    if pred[goal] is None and start != goal:
        if start == goal:
            return [start]
        return None
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        if cur == start:
            break
        cur = pred[cur]
    path.reverse()
    if path[0] != start:
        return None
    return path

# ----------------------------
# DFS - Busca em profundidade
# ----------------------------
def dfs_recursive(graph: Graph, start, visited=None, pred=None, order=None):
    """
    DFS recursiva a partir de 'start'. Retorna visited (set), pred (lista) e order (lista da ordem de entrada).
    Pode ser usada para percorrer arvores ou detectar componentes.
    """
    if visited is None:
        visited = set()
    if pred is None:
        pred = [None] * graph.n
    if order is None:
        order = []

    visited.add(start)
    order.append(start)
    for v, w in graph.neighbors(start):
        if v not in visited:
            pred[v] = start
            dfs_recursive(graph, v, visited, pred, order)
    return visited, pred, order

def dfs_iterative(graph: Graph, start):
    """
    DFS iterativa usando pilha. Retorna order de visita e predecessores.
    """
    visited = set()
    pred = [None] * graph.n
    order = []
    stack = [start]

    while stack:
        u = stack.pop()
        if u in visited:
            continue
        visited.add(u)
        order.append(u)
        # empilhar vizinhos em ordem inversa se quiser ordem predefinida
        for v, w in reversed(graph.neighbors(u)):
            if v not in visited:
                pred[v] = u
                stack.append(v)
    return visited, pred, order

# ----------------------------
# Dijkstra - Caminho minimo (pesos nao-negativos)
# ----------------------------
def dijkstra(graph: Graph, source):
    """
    Dijkstra com heap (fila de prioridade).
    Retorna (dist, pred) onde dist[u] = distancia minima de source a u (float('inf') se nao alcancavel)
    pred[u] = predecessor no caminho minimo ou None.
    Requer que todos os pesos sejam numericos e nao-negativos.
    """
    n = graph.n
    dist = [math.inf] * n
    pred = [None] * n
    dist[source] = 0

    heap = [(0, source)]
    while heap:
        d_u, u = heapq.heappop(heap)
        if d_u > dist[u]:
            continue
        for v, w in graph.neighbors(u):
            # se w for None, considerar peso implicito 1
            w_eff = 1 if w is None else w
            if w_eff < 0:
                raise ValueError("Dijkstra requer pesos nao-negativos (encontrado peso negativo).")
            if dist[v] > dist[u] + w_eff:
                dist[v] = dist[u] + w_eff
                pred[v] = u
                heapq.heappush(heap, (dist[v], v))
    return dist, pred

# ----------------------------
# Bellman-Ford - Caminho minimo com possiveis pesos negativos
# ----------------------------
def bellman_ford(graph: Graph, source):
    """
    Bellman-Ford retorna (dist, pred, has_negative_cycle)
    - dist: lista de distancias (inf se nao alcancavel)
    - pred: lista de predecessores
    - has_negative_cycle: True se existe ciclo de soma negativa atingivel do source
    Complexidade: O(V*E)
    """
    n = graph.n
    dist = [math.inf] * n
    pred = [None] * n
    dist[source] = 0

    # relaxar arestas V-1 vezes
    for i in range(n - 1):
        updated = False
        for u in range(n):
            for v, w in graph.neighbors(u):
                w_eff = 1 if w is None else w
                if dist[u] != math.inf and dist[v] > dist[u] + w_eff:
                    dist[v] = dist[u] + w_eff
                    pred[v] = u
                    updated = True
        if not updated:
            break

    # checar ciclo negativo
    has_negative_cycle = False
    for u in range(n):
        for v, w in graph.neighbors(u):
            w_eff = 1 if w is None else w
            if dist[u] != math.inf and dist[v] > dist[u] + w_eff:
                has_negative_cycle = True
                break
        if has_negative_cycle:
            break

    return dist, pred, has_negative_cycle

# ----------------------------
# Funcoes utilitarias de demonstracao
# ----------------------------
def print_path_info(path):
    if path is None:
        print("Caminho: nao existente")
    else:
        print("Caminho (vertices):", " -> ".join(map(str, path)))

def demo_bfs():
    """
    Demo BFS:
    Simulacao: rede de contatos (grafo nao ponderado).
    Objetivo: encontrar menor numero de saltos (arestas) entre usuario A e usuario B.
    Grafo contem 16 vertices (0..15).
    """
    print("\n=== DEMO BFS ===")
    n = 16
    g = Graph(n, directed=False)

    # Construir um grafo de conexoes (exemplo arbitrario, conectado)
    edges = [
        (0,1),(0,2),(1,3),(1,4),(2,5),(2,6),(3,7),(4,7),(5,8),(6,9),
        (7,10),(8,10),(9,11),(10,12),(11,13),(12,14),(13,14),(14,15)
    ]
    for u,v in edges:
        g.add_edge(u,v)

    start = 0
    target = 15
    print(f"Grafo com {n} vertices. Buscando menor numero de arestas de {start} até {target}...")
    dist, pred = bfs(g, start)
    print(f"Distancia (numero de arestas) do vertice {start} ate cada vertice:")
    print(dist)
    path = reconstruct_path(pred, start, target)
    print_path_info(path)
    # resultado: mostra caminho minimo em termos de arestas

def demo_dfs():
    """
    Demo DFS:
    Simulacao: exploracao de um labirinto representado por um grafo (pode ser tratado como arvore).
    Objetivo: fazer percurso em profundidade e listar ordem de visita, alem de detectar componentes.
    Grafos com 16 vertices.
    """
    print("\n=== DEMO DFS ===")
    n = 16
    g = Graph(n, directed=False)

    # Criar um grafo que contem duas componentes (exemplo)
    comp1 = [(0,1),(1,2),(2,3),(3,4),(1,5),(5,6),(6,7)]
    comp2 = [(8,9),(9,10),(10,11),(11,12),(12,13),(13,14),(14,15)]

    for (u,v) in comp1 + comp2:
        g.add_edge(u,v)

    # DFS recursiva a partir do vertice 0 (componente 1)
    visited1, pred1, order1 = dfs_recursive(g, 0)
    print("Componente iniciando em 0 - vertices visitados (DFS recursiva):", sorted(list(visited1)))
    print("Ordem de visita (recursiva):", order1)

    # DFS iterativa a partir de 8 (componente 2)
    visited2, pred2, order2 = dfs_iterative(g, 8)
    print("Componente iniciando em 8 - vertices visitados (DFS iterativa):", sorted(list(visited2)))
    print("Ordem de visita (iterativa):", order2)

    # Detectar componentes para todo o grafo
    all_visited = set()
    components = []
    for v in range(n):
        if v not in all_visited:
            vis, pred, order = dfs_recursive(g, v)
            components.append(sorted(list(vis)))
            all_visited.update(vis)
    print("Componentes do grafo:", components)

def demo_dijkstra():
    """
    Demo Dijkstra:
    Simulacao: malha de ruas (grafo ponderado, pesos positivos representando distancias)
    Objetivo: encontrar menor distancia entre duas localizacoes.
    Grafos com 16 vertices.
    """
    print("\n=== DEMO DIJKSTRA ===")
    n = 16
    g = Graph(n, directed=False)

    # Montar um grafo ponderado (exemplo de ruas)
    weighted_edges = [
        (0,1,4),(0,2,2),(1,2,1),(1,3,5),(2,3,8),(2,4,10),(3,5,2),(4,5,3),
        (4,6,6),(5,7,4),(6,7,1),(7,8,7),(6,9,12),(8,10,5),(9,10,3),(10,11,2),
        (11,12,4),(12,13,6),(11,13,1),(13,14,2),(14,15,3)
    ]
    # adiciona as arestas; se houver vertices nao usados eles ficam isolados, mas aqui usamos 0..15
    for u,v,w in weighted_edges:
        g.add_edge(u,v,w)

    source = 0
    target = 15
    print(f"Grafo com pesos positivos. Encontrando menor distancia de {source} ate {target} usando Dijkstra...")
    dist, pred = dijkstra(g, source)
    if dist[target] == math.inf:
        print("Alvo nao alcancavel.")
    else:
        print(f"Distancia minima de {source} ate {target}: {dist[target]:.2f}")
        path = reconstruct_path(pred, source, target)
        print_path_info(path)
    # Imprimir distancias para primeiros vertices
    print("Distancias do source para cada vertice (alguns exemplos):")
    for i in range(min(n, 16)):
        d = dist[i]
        if d == math.inf:
            print(f"{i}: inf")
        else:
            print(f"{i}: {d}")

def demo_bellman_ford():
    """
    Demo Bellman-Ford:
    Simulacao: problema com ajustes de custo que podem ser negativos (por exemplo, desconto ou ajuste).
    Objetivo: encontrar menor custo e detectar ciclo negativo se existir.
    Grafos com 16 vertices.
    """
    print("\n=== DEMO BELLMAN-FORD ===")
    n = 16
    g = Graph(n, directed=True)  # usar orientado para exemplificar
    # Montar grafo ponderado com alguns pesos negativos, mas sem ciclo negativo atingivel
    edges = [
        (0,1,6),(0,2,7),(1,2,8),(1,3,5),(1,4,-4),
        (2,3,-3),(2,4,9),(3,1,-2),(4,3,7),(3,5,2),
        (5,6,1),(6,7,3),(7,8,2),(8,9,4),(9,10,1),(10,11,2),
        (11,12,2),(12,13,3),(13,14,1),(14,15,5)
    ]
    for u,v,w in edges:
        g.add_edge(u,v,w)

    source = 0
    print(f"Executando Bellman-Ford a partir do vertice {source}...")
    dist, pred, has_neg_cycle = bellman_ford(g, source)
    if has_neg_cycle:
        print("Ciclo negativo detectado. Solucao de caminhos minima nao e confiavel.")
    else:
        print("Nenhum ciclo negativo detectado.")
        print("Algumas distancias encontradas:")
        for i in range(n):
            if dist[i] == math.inf:
                print(f"{i}: inf")
            else:
                print(f"{i}: {dist[i]}")
        target = 6
        path = reconstruct_path(pred, source, target)
        print(f"Caminho minimo (se existir) de {source} ate {target}:")
        print_path_info(path)

# ----------------------------
# Funcoes auxiliares de testes unitarios simples
# ----------------------------
def unit_tests():
    """
    Testes rapidos para verificar consistencia das implementacoes.
    """
    print("\n=== TESTES RAPIDOS ===")
    # Testar BFS em grafo pequeno
    g = Graph(4, directed=False)
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,3)
    dist, pred = bfs(g, 0)
    assert dist == [0,1,2,3], "BFS falhou em grafo linear"
    path = reconstruct_path(pred, 0, 3)
    assert path == [0,1,2,3], "Reconstrucao de caminho BFS falhou"

    # Testar DFS
    visited, pred, order = dfs_recursive(g, 0)
    assert set(visited) == {0,1,2,3}, "DFS recursiva falhou"

    # Testar Dijkstra (pesos nao-negativos)
    gd = Graph(3, directed=False)
    gd.add_edge(0,1,1); gd.add_edge(1,2,2); gd.add_edge(0,2,10)
    dist_d, pred_d = dijkstra(gd, 0)
    assert dist_d[2] == 3, "Dijkstra falhou"

    # Testar Bellman-Ford com negativo mas sem ciclo
    gb = Graph(3, directed=True)
    gb.add_edge(0,1,4); gb.add_edge(1,2,-2); gb.add_edge(0,2,10)
    dist_b, pred_b, neg = bellman_ford(gb, 0)
    assert not neg and dist_b[2] == 2, "Bellman-Ford falhou"

    print("Todos os testes rapidos passaram.")

# ----------------------------
# Menu / Execucao principal
# ----------------------------
def main():
    print("Trabalho: Caminhamentos em Grafos e Arvores - Implementacoes em Python")
    print("Executando demonstracoes automaticamente...\n")
    # Executa testes unitarios
    unit_tests()

    # Executa demos
    demo_bfs()
    demo_dfs()
    demo_dijkstra()
    demo_bellman_ford()

    print("\nFIM das demonstracoes.")
    print("Observacoes finais:")
    print("- BFS e DFS sao apropriados para busca e exploracao (BFS encontra menor numero de arestas; DFS explora profundamente).")
    print("- Dijkstra exige pesos nao-negativos e e otimizado com heap (O(E log V)).")
    print("- Bellman-Ford aceita pesos negativos e detecta ciclos negativos (O(V*E)).")
    print("- Para grafos muito grandes, considerar estruturas e implementacoes otimizadas e heuristicas (ex.: A*, bidirectional search).")

if __name__ == "__main__":
    main()
