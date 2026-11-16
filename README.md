# Relatório — Pathfinding A* no S3

## 1. Objetivo

Implementámos o algoritmo A* na classe `s3.ai.AStar`, e com isso as unidades do S3 passaram a mover-se. Antes, o jogo arrancava mas nenhuma unidade saía do lugar: ao dar uma ordem de movimento, aparecia "Can't get to that location." e as IAs ficavam paradas.

Isto acontecia porque o método `computePath()` devolvia sempre `null`. Todas as unidades móveis (camponeses, soldados, arqueiros, cavaleiros e catapultas) pedem um caminho a esta classe antes de andar. O enunciado pedia apenas que completássemos essa classe.

## 2. Como funciona o A*

O mapa do S3 é uma grelha de células. O A* encontra o caminho mais curto entre duas células porque, de todas as células que ainda pode visitar, explora primeiro a que parece mais promissora. Para escolher essa célula, calcula três valores:

- **g**: quantos passos já foram dados desde o início. Cada passo custa 1.
- **h** (heurística): uma estimativa dos passos que faltam até ao destino. Usámos a distância de Manhattan, |x − x<sub>destino</sub>| + |y − y<sub>destino</sub>|.
- **f = g + h**: o custo total estimado. A célula com o menor f é a próxima a ser explorada.

O algoritmo mantém duas listas:

1. A **lista aberta** guarda as células descobertas que ainda faltam explorar. Usámos uma fila de prioridade, que devolve sempre a célula com o menor f.
2. A **lista fechada** guarda as células já exploradas, para nunca serem visitadas duas vezes.

Em cada passo, o algoritmo retira a melhor célula da lista aberta e olha para as 4 vizinhas (cima, baixo, esquerda e direita). Cada vizinha livre é acrescentada à lista, junto com a célula de onde se veio (o "pai"). Quando o destino sai da lista, reconstrói-se o caminho seguindo os pais de trás para a frente.

Como as unidades só andam em 4 direções, a distância de Manhattan nunca sobrestima os passos que faltam. Por isso, o caminho encontrado é sempre o mais curto possível.

## 3. Código antes e depois

**Antes.** A classe só tinha os esqueletos dos métodos, e por isso nunca havia caminho:

```java
public AStar(double start_x, double start_y, double goal_x, double goal_y,
             S3PhysicalEntity i_entity, S3 the_game) {
    // ...
}

public List<Pair<Double, Double>> computePath() {
    // ...
    return null;
}
```

**Depois.** O construtor passou a guardar a origem, o destino, a unidade e o jogo. O `computePath()` passou a ter o ciclo do A* (versão resumida, sem comentários de linha):

```java
public List<Pair<Double, Double>> computePath() {
    int width = game.getMap().getWidth(), height = game.getMap().getHeight();
    if (!inside(goalX, goalY, width, height)) return null;                  // destino fora do mapa
    if (startX == goalX && startY == goalY) return new LinkedList<>();      // já lá está

    S3PhysicalEntity probe = (S3PhysicalEntity) entity.clone();             // cópia para testar células
    int[] g = new int[width * height];      Arrays.fill(g, Integer.MAX_VALUE);
    int[] parent = new int[width * height]; Arrays.fill(parent, -1);
    boolean[] closed = new boolean[width * height];
    PriorityQueue<int[]> open = new PriorityQueue<>(/* menor f; empate: menor h */);

    int start = startY * width + startX, goal = goalY * width + goalX;
    g[start] = 0;
    open.add(new int[]{ start, heuristic(startX, startY), heuristic(startX, startY) });

    while (!open.isEmpty()) {
        int current = open.poll()[0];
        if (closed[current]) continue;
        closed[current] = true;
        if (current == goal) return buildPath(parent, goal, width);

        for (int d = 0; d < 4; d++) {                                       // 4 vizinhos
            int nx = current % width + DX[d], ny = current / width + DY[d];
            if (!inside(nx, ny, width, height)) continue;
            int next = ny * width + nx;
            if (closed[next] || !isFree(probe, nx, ny)) continue;
            int newG = g[current] + 1;
            if (newG < g[next]) {                                           // caminho melhor
                g[next] = newG;  parent[next] = current;
                int h = heuristic(nx, ny);
                open.add(new int[]{ next, newG + h, h });
            }
        }
    }
    return null;                                                            // não há caminho
}
```

Três métodos auxiliares completam a classe:

- `heuristic(x, y)` devolve a distância de Manhattan até ao destino.
- `isFree(probe, x, y)` põe a cópia da unidade na célula e pergunta a `game.anyLevelCollision()` se ela choca com alguma coisa.
- `buildPath(...)` segue os pais desde o destino até à origem e insere cada célula no início da lista. Assim, o caminho fica na ordem certa, da origem para o destino.

## 4. Decisões e detalhes

| Decisão | Porquê |
|---|---|
| Testar células com `anyLevelCollision` numa cópia da unidade (`clone()`) | É a função recomendada no enunciado e deteta árvores, água, edifícios e outras unidades. A cópia tem o mesmo ID, por isso a unidade não choca consigo própria, e a unidade real não é mexida durante o cálculo. |
| Movimento em 4 direções, com custo 1 por passo | As unidades do S3 andam uma célula de cada vez. Sem diagonais, nunca "cortam" cantos de árvores ou edifícios. |
| Heurística de Manhattan | Nunca sobrestima a distância com 4 direções, o que garante o caminho mais curto. |
| Empate em f resolvido pelo menor h | Quando duas células têm o mesmo f, explora-se primeiro a mais perto do destino. Assim, o algoritmo visita menos células. |
| Arrays indexados por `y * largura + x` | São mais rápidos e simples do que guardar objetos de nó num mapa. |
| Caminho sem a origem, com o destino, por ordem, e `null` se não houver caminho | São as regras do enunciado, e é o que `WTroop.moveTowardsTarget()` espera receber. |

Também mudámos `REDRAWING_PERIOD` em `S3App` de 1 para 20 ms por frame, como sugere o enunciado. Com 1 ms, o jogo corre rápido demais para um humano jogar. Por fim, configurámos o projeto no VS Code (pasta `.vscode`) para incluir as bibliotecas da pasta `libs/`.

## 5. Testes e resultados

Com o A* implementado, as unidades andam, contornam obstáculos, recolhem recursos, constroem e combatem, e as IAs chegam ao fim dos jogos.
