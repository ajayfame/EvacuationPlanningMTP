package com.iitb.gise;

import java.util.ArrayList;

public class Graph {
	
	private ArrayList<Edge> edgeList;
	private ArrayList<Node> nodeList;
	
	public Graph()
	{
		edgeList = new ArrayList<Edge>();
		nodeList = new ArrayList<Node>();
	}
	public ArrayList<Edge> getEdgeList() {
		return edgeList;
	}
	public void setEdgeList(ArrayList<Edge> edgeList) {
		this.edgeList = edgeList;
	}
	public ArrayList<Node> getNodeList() {
		return nodeList;
	}
	public void setNodeList(ArrayList<Node> nodeList) {
		this.nodeList = nodeList;
	}
	
	public void addNode(Node node)
	{
		this.getNodeList().add(node);
	}
	
	public void addEdge(Edge edge)
	{
		this.getEdgeList().add(edge);
	}
	
	public void clearNodeData()
	{
		for(int i=0;i<nodeList.size();i++)
		{
			Node n = nodeList.get(i);
			n.setChildList(null);
			n.setParent(null);
			n.setPathUptoPreviousNode(null);
			n.setScanned(false);
			n.setAdjacentScannedList(null);
			if(n.getNodeType() == Node.SOURCE)
				n.setTravelTime(0);
			else
				n.setTravelTime(Double.MAX_VALUE);
		}
	}
}