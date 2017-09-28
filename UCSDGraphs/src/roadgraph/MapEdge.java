package roadgraph;

import geography.GeographicPoint;

public class MapEdge {

	private GeographicPoint startLoc;
	private GeographicPoint endLoc;
	private String streetName;
	private Double length;
	private String streetType;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, String name, 
					Double distance, String roadType) {
		this.startLoc = start;
		this.endLoc = end;
		this.streetName = name;
		this.length = distance;
		this.streetType = roadType;
		
	}

	/**
	 * @return the startLoc
	 */
	public GeographicPoint getStartLoc() {
		return startLoc;
	}

	/**
	 * @param startLoc the startLoc to set
	 */
	public void setStartLoc(GeographicPoint startLoc) {
		this.startLoc = startLoc;
	}

	/**
	 * @return the endLoc
	 */
	public GeographicPoint getEndLoc() {
		return endLoc;
	}

	/**
	 * @param endLoc the endLoc to set
	 */
	public void setEndLoc(GeographicPoint endLoc) {
		this.endLoc = endLoc;
	}

	/**
	 * @return the streetName
	 */
	public String getStreetName() {
		return streetName;
	}

	/**
	 * @param streetName the streetName to set
	 */
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}

	/**
	 * @return the length
	 */
	public Double getLength() {
		return length;
	}

	/**
	 * @param length the length to set
	 */
	public void setLength(Double length) {
		this.length = length;
	}

	/**
	 * @return the streetType
	 */
	public String getStreetType() {
		return streetType;
	}

	/**
	 * @param streetType the streetType to set
	 */
	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}
	
public String toString() {
	return (getStartLoc() + " " + getEndLoc()  +" Length: " + this.getLength());
}
}
