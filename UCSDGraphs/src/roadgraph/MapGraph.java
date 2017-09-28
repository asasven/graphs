/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint, MapVertex> vertices;
	private List<MapEdge> edges;
	private Map<MapVertex, Double> distances;


	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
	
		this.vertices = new HashMap<GeographicPoint, MapVertex>();
		this.edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
		
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
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
		// TODO: Implement this method in WEEK 3
		if ( vertices.containsKey(location)|| location == null ) {
			return false;
		}
		else {
			MapVertex v = new MapVertex(location);
			vertices.put(location, v);
			return true;
		}
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

		//TODO: Implement this method in WEEK 3
		if ( vertices.containsKey(from) && 
			vertices.containsKey(to) &&
			roadType != null &&
			length > 0
			) {
			MapEdge e = new MapEdge(from, to, roadName, length, roadType);
			edges.add(e);
			vertices.get(from).addEdge(e);
			
		}
		else 
			throw new IllegalArgumentException();
		
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
		// TODO: Implement this method in WEEK 3		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Queue <GeographicPoint> queue = new LinkedList <GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		queue.add(start);
		GeographicPoint curr = start;
		nodeSearched.accept(curr);
		queue.add(start);
		visited.add(start);
		while (!queue.isEmpty()) {
			curr = queue.remove();
			nodeSearched.accept(curr);
			if (curr.equals(goal)) {
				
				return usedPath(vertices.get(curr));
				}
			else {
				
				List <GeographicPoint> neighbors = vertices.get(curr).getNeighbors();
				for(GeographicPoint n: neighbors) {
					if (! visited.contains(n)) {
						visited.add(n);
						vertices.get(n).setPrevVertexInPath(vertices.get(curr));
						queue.add(n);						
					}
				}
			}
		}	
		return null;
	}
	
	/** Traces the used path back from goal 
	 * 
	 * */
	public List<GeographicPoint> usedPath(MapVertex goal){
		
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(goal.getLocation());
		MapVertex target = goal;
		  while (target.getPrevVertexInPath() != null) {
			    target = target.getPrevVertexInPath();
			    path.add(target.getLocation());
			  }
			  Collections.reverse(path); 
			  return path;
	}
	
	private List<GeographicPoint>
	reconstructPath(HashMap<MapVertex,MapVertex> parentMap,
			MapVertex start, MapVertex goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapVertex current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}
	
	  public  Comparator<MapVertex> vertComp = new Comparator<MapVertex>(){

	        @Override
	        public int compare(MapVertex o1, MapVertex o2) {
	            return (int) ( getShortestDistance(o1) - getShortestDistance(o2));
	        }

	    };
	

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
		
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapVertex startNode = vertices.get(start);
		MapVertex endNode = vertices.get(goal);
		HashMap<MapVertex,MapVertex> parentMap = new HashMap<MapVertex,MapVertex>();
		PriorityQueue<MapVertex> queue = new PriorityQueue<MapVertex>(vertComp);
		Set<MapVertex> visited = new HashSet<MapVertex>();

		initDistances();
		MapVertex curr = startNode;
		distances.put(curr, 0.00);
		queue.add(curr);

		//nodeSearched.accept(start);

		while (!queue.isEmpty()) {
			curr = queue.poll();
			System.out.println("Dijkstra: out from queue:" + curr);
			if(! visited.contains(curr)) {
				visited.add(curr);
				if (curr.equals(endNode) ){
					//return usedPath(curr);
					break;
				}

				List <GeographicPoint> neighbors = curr.getNeighbors();
				for(GeographicPoint n: neighbors) {
					MapVertex nv = vertices.get(n);
					if (! visited.contains(nv)) {
						Double dd = getShortestDistance(nv);
						System.out.println("vert:"  + nv + "dist:" + dd);
						if (dd  >
						(getShortestDistance(curr) + curr.getNeighborDistance(nv)  )
								) {

							distances.put(nv,curr.getNeighborDistance(nv) + getShortestDistance(curr));
							nv.setPrevVertexInPath(curr);
							parentMap.put(nv, curr);
							queue.add(nv);	
						}
					}
				}
			}
		}	
		//return null;
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;
	}
	
	private void initDistances() {
		this.distances = new HashMap<MapVertex, Double>();
	}
	
    private Double getShortestDistance(MapVertex destination) {
    		Double d = distances.get(destination);
        if (d == null) {
            return Double.MAX_VALUE;
        } else {
            return d;
        }
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		PriorityQueue<MapVertex> queue = new PriorityQueue<MapVertex>(vertComp);
		Set<MapVertex> visited = new HashSet<MapVertex>();

		initDistances();
		MapVertex curr = vertices.get(start);	
		distances.put(curr, 0.00);
		queue.add(curr);

		//nodeSearched.accept(start);

		while (!queue.isEmpty()) {
			//System.out.println("*******queue:" + queue);
			curr = queue.poll();
			System.out.println("Astar: out from queue:" + curr);
			if(! visited.contains(curr)) {
				visited.add(curr);
				if (curr.equals(vertices.get(goal)) ){
					return usedPath(curr);
				}

				List <GeographicPoint> neighbors = curr.getNeighbors();
				for(GeographicPoint n: neighbors) {
					MapVertex nv = vertices.get(n);
					if (! visited.contains(nv)) {

						if ((getShortestDistance(nv)+ nv.getBirdDistanceTo(vertices.get(goal))) > 
						(getShortestDistance(curr) + curr.getNeighborDistance(nv) 			
								) ) {

							distances.put(nv,curr.getNeighborDistance(nv) + 
									getShortestDistance(curr) + nv.getBirdDistanceTo(vertices.get(goal))
							);
							nv.setPrevVertexInPath(curr);
							queue.add(nv);	
						}
					}
				}
			}
		}	
		return null;
	}

	public void printGraph() {
		System.out.print( "num vertices: " +this.getNumVertices());
		System.out.println( " num edges: " +this.getNumEdges());
	
		for (MapVertex vert : vertices.values()) {
			System.out.println( vert.toString());
			for(MapEdge e: vert.getEdges()) {
				System.out.println( "Edge: " + e.toString());
			}
		}
	}
	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		//firstMap.printGraph();
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		//List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		//	System.out.println("testroute:" + testroute);
		
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute1 = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println("******************Dijkstratestroute1:" + testroute1);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("******************a star testroute2:" + testroute2);
	
			
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		List<GeographicPoint> testroute3 = testMap.dijkstra(testStart,testEnd);
		System.out.println("******************Dijkstra test route3:" + testroute3);
		//testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			/*
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*	
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		System.out.println("********************quiz test");
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 */
		
		
	}
	
}
