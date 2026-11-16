package s3.ai;


import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

import s3.base.S3;
import s3.entities.S3PhysicalEntity;
import s3.util.Pair;


/**
 * A* pathfinding over the S3 cell grid.
 *
 * - Each map cell is a node; a unit moves to its 4 neighbours (up, down, left, right).
 * - Every step costs 1 (g). The heuristic (h) is the Manhattan distance to the goal,
 *   which never overestimates with 4-way movement, so the path found is the shortest.
 * - A cell is free when game.anyLevelCollision() finds nothing there (no tree, water,
 *   building or other unit).
 * - The returned path does NOT include the start, DOES include the goal, goes from
 *   start to goal, and is null when no path exists.
 */
public class AStar {
	
	// 4-way movement: right, left, down, up
	private static final int[] DX = { 1, -1, 0, 0 };
	private static final int[] DY = { 0, 0, 1, -1 };

	private final int startX, startY, goalX, goalY;
	private final S3PhysicalEntity entity;
	private final S3 game;

	public static int pathDistance(double start_x, double start_y, double goal_x, double goal_y,
			S3PhysicalEntity i_entity, S3 the_game) {
		AStar a = new AStar(start_x,start_y,goal_x,goal_y,i_entity,the_game);
		List<Pair<Double, Double>> path = a.computePath();
		if (path!=null) return path.size();
		return -1;
	}

	public AStar(double start_x, double start_y, double goal_x, double goal_y,
			S3PhysicalEntity i_entity, S3 the_game) {
		// Coordinates are doubles, but the map is a grid of whole cells
		this.startX = (int) start_x;
		this.startY = (int) start_y;
		this.goalX = (int) goal_x;
		this.goalY = (int) goal_y;
		this.entity = i_entity;
		this.game = the_game;
	}

	public List<Pair<Double, Double>> computePath() {
		// Demo switch: run with -Dastar.disabled=true to see the game as it was before A* (always null)
		if (Boolean.getBoolean("astar.disabled")) return null;

		int width = game.getMap().getWidth();
		int height = game.getMap().getHeight();

		// Goal outside the map (e.g. no free cell next to a target) -> no path
		if (!inside(goalX, goalY, width, height)) return null;

		// Already there: nothing to walk
		if (startX == goalX && startY == goalY) return new LinkedList<Pair<Double, Double>>();

		// A copy of the unit used to test cells without moving the real one
		S3PhysicalEntity probe = (S3PhysicalEntity) entity.clone();

		// One slot per cell (index = y * width + x)
		int size = width * height;
		int[] g = new int[size];          // cost from start to this cell
		int[] parent = new int[size];     // previous cell on the best path
		boolean[] closed = new boolean[size];
		Arrays.fill(g, Integer.MAX_VALUE);
		Arrays.fill(parent, -1);

		// Open list ordered by f = g + h (ties broken by smaller h)
		PriorityQueue<int[]> open = new PriorityQueue<int[]>((a, b) ->
				a[1] != b[1] ? Integer.compare(a[1], b[1]) : Integer.compare(a[2], b[2]));

		int start = startY * width + startX;
		int goal = goalY * width + goalX;
		g[start] = 0;
		open.add(new int[] { start, heuristic(startX, startY), heuristic(startX, startY) });

		while (!open.isEmpty()) {
			int current = open.poll()[0];
			if (closed[current]) continue;   // old, worse copy of a cell already expanded
			closed[current] = true;

			if (current == goal) return buildPath(parent, goal, width);

			int cx = current % width;
			int cy = current / width;

			for (int d = 0; d < 4; d++) {
				int nx = cx + DX[d];
				int ny = cy + DY[d];
				if (!inside(nx, ny, width, height)) continue;

				int next = ny * width + nx;
				if (closed[next]) continue;
				if (!isFree(probe, nx, ny)) continue;

				int newG = g[current] + 1;
				if (newG < g[next]) {          // found a better way to reach this cell
					g[next] = newG;
					parent[next] = current;
					int h = heuristic(nx, ny);
					open.add(new int[] { next, newG + h, h });
				}
			}
		}

		return null; // every reachable cell was explored and the goal was not among them
	}

	/** Manhattan distance to the goal. */
	private int heuristic(int x, int y) {
		return Math.abs(x - goalX) + Math.abs(y - goalY);
	}

	private static boolean inside(int x, int y, int width, int height) {
		return x >= 0 && y >= 0 && x < width && y < height;
	}

	/** A cell is free when the unit placed there collides with nothing. */
	private boolean isFree(S3PhysicalEntity probe, int x, int y) {
		probe.setX(x);
		probe.setY(y);
		return game.anyLevelCollision(probe) == null;
	}

	/** Walks the parent links back from the goal, then returns them start -> goal (start excluded). */
	private static List<Pair<Double, Double>> buildPath(int[] parent, int goal, int width) {
		LinkedList<Pair<Double, Double>> path = new LinkedList<Pair<Double, Double>>();
		int cell = goal;
		while (parent[cell] != -1) {
			path.addFirst(new Pair<Double, Double>((double) (cell % width), (double) (cell / width)));
			cell = parent[cell];
		}
		return path;
	}

}
