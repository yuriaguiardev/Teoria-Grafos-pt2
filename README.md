 Trabalho de Grafos – Implementação de Caminhamentos e Algoritmos de Menor Caminho

Este projeto implementa, em Python, os principais algoritmos de caminhamento em grafos e algoritmos clássicos de menor caminho. Inclui demonstrações completas no terminal e foi desenvolvido para fins acadêmicos.

 Estrutura do Projeto
/trabalho-grafos
│
├── main.py        # Arquivo principal contendo todo o código
└── README.md      # Este documento

 Como Executar
1. Requisitos

Python 3.8 ou superior

Nenhuma biblioteca externa é necessária (somente módulos padrão)

2. Executar o programa

Abra o terminal na pasta do projeto e execute:

python main.py


O programa automaticamente exibirá exemplos e testes dos algoritmos.

 Algoritmos Implementados
 BFS — Breadth-First Search

Percorre o grafo por camadas, utilizando fila.

Encontra o menor número de arestas entre dois vértices.

Retorna:

ordem de visita,

distâncias,

reconstrução do caminho.

 DFS — Depth-First Search

Implementado em duas versões:

Recursiva

Iterativa (com pilha)

Utilizado para:

exploração profunda,

análise estrutural,

identificação de componentes,

uso inicial para ordenação topológica.

 Dijkstra

Calcula o menor caminho em grafos ponderados sem pesos negativos.

Utiliza fila de prioridade (heapq).

Retorna:

vetor de distâncias,

caminho mínimo reconstruído.

 Bellman–Ford

Aceita pesos negativos.

Detecta ciclos negativos.

Retorna:

distâncias,

caminho reconstruído,

alerta sobre possível ciclo negativo.

 Estrutura dos Grafos Utilizados

O projeto trabalha com:

um grafo não ponderado (para BFS e DFS),

um grafo ponderado com pesos positivos (para Dijkstra),

um grafo com pesos negativos (para Bellman-Ford).

Todos definidos diretamente em main.py para facilitar a correção.

O grafo contém pelo menos 16 vértices, cumprindo as exigências acadêmicas.

 Demonstrações ao Executar

O programa automaticamente exibe:

Saída do BFS

Saída do DFS (recursivo e iterativo)

Cálculo do menor caminho com Dijkstra

Execução do Bellman-Ford com verificação de ciclos

Exemplo:

=== DEMO BFS ===
Distâncias encontradas: [0, 1, 1, 2, 2, 3, ...]

=== DEMO DIJKSTRA ===
Caminho mínimo de 0 até 15: 0 -> 2 -> 1 -> 8 -> 15
Distância: 34.0

=== DEMO BELLMAN-FORD ===
Nenhum ciclo negativo detectado.

 Objetivo do Projeto

Este projeto foi criado para fins educacionais, permitindo ao estudante:

compreender o funcionamento dos algoritmos,

visualizar suas aplicações,

comparar limitações entre BFS/DFS/Dijkstra/Bellman-Ford,

ter um código simples, organizado e amplamente comentado.

