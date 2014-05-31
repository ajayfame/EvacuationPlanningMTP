package com.iitb.gise;

import java.util.ArrayList;

public class PathUptoNode {

	ArrayList<Node> nodeList;
	ArrayList<Edge> edgeList;
	ArrayList<Integer> departureTime;
	
	public PathUptoNode()
	{
		nodeList = new ArrayList<Node>();
		edgeList = new ArrayList<Edge>();
		departureTime = new ArrayList<Integer>();
	}
	public PathUptoNode(PathUptoNode pNode)
	{
		if(pNode != null)
		{
			for(int i=0; i<pNode.getNodeList().size(); i++)
			{
				if(this.nodeList == null)
					this.nodeList = new ArrayList<Node>();
				if(this.edgeList == null)
					this.edgeList = new ArrayList<Edge>();
				if(this.departureTime == null)
					this.departureTime = new ArrayList<Integer>();
				
				this.nodeList.add(pNode.getNodeList().get(i));
				this.edgeList.add(pNode.getEdgeList().get(i));
				this.departureTime.add(pNode.getDepartureTime().get(i));
			}
		}
	}
	public ArrayList<Node> getNodeList() {
		return nodeList;
	}
	public void setNodeList(ArrayList<Node> nodeList) {
		this.nodeList = nodeList;
	}
	public ArrayList<Integer> getDepartureTime() {
		return departureTime;
	}
	public void setDepartureTime(ArrayList<Integer> departureTime) {
		this.departureTime = departureTime;
	}
	
	public ArrayList<Edge> getEdgeList() {
		return edgeList;
	}
	public void setEdgeList(ArrayList<Edge> edgeList) {
		this.edgeList = edgeList;
	}
	public void add(Node node, Edge edge, int departureTime)
	{
		this.nodeList.add(node);
		this.edgeList.add(edge);
		this.departureTime.add(departureTime);
	}
}