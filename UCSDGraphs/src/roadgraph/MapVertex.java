package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapVertex  {
	private GeographicPoint location;
	private ArrayList<MapEdge> edges;
	private MapVertex prevVertexInPath;
	//private double distanceToStart; 
	

	
	public MapVertex(GeographicPoint gp) {
		this.location = gp;
		this.edges = new ArrayList<MapEdge>();
	}
	
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	public List<GeographicPoint> getNeighbors() {
		List<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();
		for (MapEdge e: edges) {
			neighbors.add(e.getEndLoc());
		
		}
		return neighbors;
	}
	
	public Double getNeighborDistance(MapVertex start) {
		for (MapEdge e: this.edges) {
			//System.out.println("lenght:"  + e.getLength() + "start:" + start.getLocation() + "edge loc:" + e.getEndLoc());
			if (e.getEndLoc().equals(start.getLocation())) return e.getLength();
		
		}
		return null;
	}
	
	public Double getBirdDistanceTo(MapVertex goal) {
		return this.location.distance(goal.getLocation());
	}
	

	
	public String toString() {
		return (location.toString() 
				//"  edges:"  + edges.size()  + " neigh: " + getNeighbors()
				);
	}
	
	/**
	 * @return the location
	 */
	public GeographicPoint getLocation() {
		return location;
	}

	/**
	 * @param location the location to set
	 */
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	/**
	 * @return the edges
	 */
	public ArrayList<MapEdge> getEdges() {
		return edges;
	}

	/**
	 * @param edges the edges to set
	 */
	public void setEdges(ArrayList<MapEdge> edges) {
		this.edges = edges;
	}

	/**
	 * @return the prevVertexInPath
	 */
	public MapVertex getPrevVertexInPath() {
		return prevVertexInPath;
	}

	/**
	 * @param prevVertexInPath the prevVertexInPath to set
	 */
	public void setPrevVertexInPath(MapVertex prevVertexInPath) {
		this.prevVertexInPath = prevVertexInPath;
	}



	


}
