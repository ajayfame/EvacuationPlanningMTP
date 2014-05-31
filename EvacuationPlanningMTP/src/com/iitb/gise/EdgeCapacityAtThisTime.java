package com.iitb.gise;

import java.util.ArrayList;
import java.util.Iterator;

/*
 *   col represents section of the edge divided wrt travelTime
 *   Basically, each entry in the array is
 *          at time instance T(row), capacity of this section of edge 
 */
public class EdgeCapacityAtThisTime {

	private ArrayList<Integer> capacityOfSection;

	public EdgeCapacityAtThisTime() {
		capacityOfSection = new ArrayList<Integer>();
	}

	public EdgeCapacityAtThisTime(EdgeCapacityAtThisTime edgeCapacityAtThisTime) 
	{
		this.capacityOfSection = new ArrayList<Integer>();
		Iterator<Integer> it = edgeCapacityAtThisTime.getCapacityOfSection().iterator();
		while (it.hasNext()) 
			this.capacityOfSection.add(it.next());
	}
	
	public ArrayList<Integer> getCapacityOfSection() {
		return capacityOfSection;
	}

	public void setCapacityOfSection(ArrayList<Integer> capacityOfSection) {
		this.capacityOfSection = capacityOfSection;
	}
}