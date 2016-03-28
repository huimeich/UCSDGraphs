/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.ListIterator;
import java.util.Set;
import java.util.function.Consumer;
import java.util.Map;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Comparator;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Map<GeographicPoint,MapGraphNode> adjListsMap;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		numVertices = 0;
		numEdges = 0;
		adjListsMap = new HashMap<GeographicPoint,MapGraphNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		//Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		//vertices = adjListsMap.keySet();
		return new HashSet<GeographicPoint>(adjListsMap.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (location == null) return false;
		if (adjListsMap.containsKey(location)) return false;
		adjListsMap.put(location,new MapGraphNode(location));
		numVertices ++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		numEdges ++;
		if (adjListsMap.containsKey(from) && adjListsMap.containsKey(to) && 
				length >= 0 && roadType != null && roadName != null) {
			MapGraphNode toNode = adjListsMap.get(to);
			(adjListsMap.get(from)).addNeighbor(toNode,roadName,roadType,length);
		}
//		else {
//			throw new IllegalArgumentException();
//		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if ((!adjListsMap.containsKey(start)) || (!adjListsMap.containsKey(goal))) {
			System.out.println("Start or goal node is not exit in graph!  No path exists.");
			return null;//new LinkedList<GeographicPoint>();
		}

		MapGraphNode startNode = adjListsMap.get(start);
		MapGraphNode goalNode = adjListsMap.get(goal);
		
		HashSet<MapGraphNode> visited = new HashSet<MapGraphNode>();
		Queue<MapGraphNode> toExplore = new LinkedList<MapGraphNode>();
		HashMap<MapGraphNode, MapGraphNode> parentMap = new HashMap<MapGraphNode, MapGraphNode>();
		toExplore.add(startNode);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapGraphNode curr = toExplore.remove();
			if ((curr.getLocation()).equals(goal)) {
				found = true;
				break;
			}
			List<MapGraphNode> neighbors = curr.getNeighbors();
			ListIterator<MapGraphNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapGraphNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
					nodeSearched.accept(next.getLocation());
				}
			}
		}

		if (!found) {
			//System.out.println("No path exists");
			return null;//new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapGraphNode curr = goalNode;
		while (curr != startNode) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if ((!adjListsMap.containsKey(start)) || (!adjListsMap.containsKey(goal))) {
			System.out.println("Start or goal node is not exit in graph!  No path exists.");
			return null;//new LinkedList<GeographicPoint>();
		}
		MapGraphNode startNode = adjListsMap.get(start);
		MapGraphNode goalNode = adjListsMap.get(goal);
//		System.out.println("start is inside adjListMap " + (startNode.getLocation()).toString());
//		System.out.println("goal is inside adjListMap " + (goalNode.getLocation()).toString());

		HashSet<MapGraphNode> visited = new HashSet<MapGraphNode>();
		Comparator<MapGraphNode> comparator = new DistanceComparator();
		PriorityQueue<MapGraphNode> toExplore = new PriorityQueue<MapGraphNode>(comparator);
		HashMap<MapGraphNode, MapGraphNode> parentMap = new HashMap<MapGraphNode, MapGraphNode>();
		toExplore.add(startNode);
		startNode.changeDistance(0.0);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapGraphNode curr = toExplore.remove();
//			curr.changeDistance(startNode.getLength(curr));
//			System.out.println("curr is " + (curr.getLocation()).toString());

			if ((curr.getLocation()).equals(goal)) {
				found = true;
				break;
			}
			List<MapGraphNode> neighbors = curr.getNeighbors();
			ListIterator<MapGraphNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapGraphNode next = it.previous();
//				System.out.println("next is " + (next.getLocation()).toString() + " length is " + curr.getLength(next));
//				System.out.println( next.getDistance() + " " + curr.getDistance() );

				if (!visited.contains(next) && next.getDistance() - curr.getLength(next) > curr.getDistance()) {
					next.changeDistance(curr.getDistance() + curr.getLength(next));
	//				System.out.println("next is updating " + (next.getLocation()).toString() + " distance " + next.getDistance());
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
					nodeSearched.accept(next.getLocation());
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return null;//new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapGraphNode curr = goalNode;
		while (curr != startNode) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
/*
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
*/		 
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
				
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		//List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		for (GeographicPoint v: route){
			System.out.println(v.toString());
		}
		
	}
	
}
