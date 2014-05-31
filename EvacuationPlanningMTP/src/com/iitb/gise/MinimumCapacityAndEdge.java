package com.iitb.gise;

public class MinimumCapacityAndEdge {

	int minCapacity;
	Edge minCutEdge;
	Node nodeToReset;
	public MinimumCapacityAndEdge()
	{
		
	}
	 public MinimumCapacityAndEdge(int minCapacity, Edge edge, Node nodeToReset)
	 {
		 this.minCapacity = minCapacity;
		 this.minCutEdge = edge;
		 this.nodeToReset = nodeToReset;
	 }
	public int getMinCapacity() {
		return minCapacity;
	}
	public void setMinCapacity(int minCapacity) {
		this.minCapacity = minCapacity;
	}
	public Edge getMinCutEdge() {
		return minCutEdge;
	}
	public void setMinCutEdge(Edge minCutEdge) {
		this.minCutEdge = minCutEdge;
	}
	public Node getNodeToReset() {
		return nodeToReset;
	}
	public void setNodeToReset(Node nodeToReset) {
		this.nodeToReset = nodeToReset;
	}
}