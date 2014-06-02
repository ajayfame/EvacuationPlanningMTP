package com.iitb.gise;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class MainProgram {
	
	private static CCRPWithReuse1 ccrp;
	private static BasicCCRP1 basicCCRP;
	private static BasicCCRP1 basicCCRP2;
	private static CCRP_PlusPlus1 ccrpPlusPlus;
	private static KiaCCRP kiaCCRP;
	private static ShelterBasedCCRP shelterBasedCCRP;
	private static OneSourceOnePath oneSourceOnePath;
	private static PreferenceToLongerPaths prefToLongerPaths;
	
	static int sum = 0;
	static int avg = 0;
	static int count = 0;
	public static void readNodeAndEdgeFiles(String nodeFile, String edgeFile)
	{
		String csvFile = nodeFile;
		BufferedReader br = null;
		String line = "";
		String cvsSplitBy = ",";
		try 
		{
	 
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				String[] stringNode = line.split(cvsSplitBy);
				String nodeId = stringNode[0];
				String nodeName = stringNode[1];
				double x = Double.parseDouble(stringNode[2]);
				double y = Double.parseDouble(stringNode[3]);
				int maxCapacity = (int)Double.parseDouble(stringNode[4]);
				int initialOccupancy = (int)Double.parseDouble(stringNode[5]);
				String nodeTypeC = stringNode[6];
				int nodeType = 2;
				if(nodeTypeC.equals("S"))
					nodeType = Node.SOURCE;
				else if(nodeTypeC.equals("D"))
					nodeType = Node.DESTINATION;
				else
					nodeType = Node.NORMAL;
				ccrp.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				basicCCRP.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				ccrpPlusPlus.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				//shelterBasedCCRP.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				oneSourceOnePath.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				prefToLongerPaths.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				
			}
			csvFile = edgeFile;
			br.close();
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				count++;
				
				String[] stringEdge = line.split(cvsSplitBy);
				int edgeID = Integer.parseInt(stringEdge[0]);
				String edgeName = stringEdge[1];
				String sourceName = stringEdge[2];
				String targetName = stringEdge[3];
				int travelTime = (int)Double.parseDouble(stringEdge[4]);
				int maxCapacity = (int)Double.parseDouble(stringEdge[5]);
				sum+=maxCapacity;
				ccrp.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				basicCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				ccrpPlusPlus.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				//shelterBasedCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				oneSourceOnePath.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				prefToLongerPaths.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
			}
		} 
		catch (FileNotFoundException e) 
		{
			e.printStackTrace();
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		} 
		finally 
		{
			if (br != null) {
				try 
				{
					br.close();
				} 
				catch (IOException e) 
				{
					e.printStackTrace();
				}
			}
		}
	}

	public static void changeNodeCap(String nodeFile)
	{
		String csvFile = nodeFile;
		BufferedReader br = null;
		String line = "";
		String cvsSplitBy = ",";
		try 
		{
	 
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				String[] stringNode = line.split(cvsSplitBy);
				String nodeName = stringNode[1];
				int initialOccupancy = (int)Double.parseDouble(stringNode[3]);
				ccrp.changeNodeCap(nodeName, initialOccupancy);				
			}
		} 
		catch (FileNotFoundException e) 
		{
			e.printStackTrace();
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		} 
		finally 
		{
			if (br != null) {
				try 
				{
					br.close();
				} 
				catch (IOException e) 
				{
					e.printStackTrace();
				}
			}
		}
	}
	
	public static void readNodeAndEdgeFiles(String nodeFile, String edgeFile, String a)
	{
		String csvFile = nodeFile;
		BufferedReader br = null;
		String line = "";
		String cvsSplitBy = ",";
		try 
		{
	 
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				String[] stringNode = line.split(cvsSplitBy);
				String nodeId = stringNode[0];
				String nodeName = stringNode[1];
				double x = Double.parseDouble(stringNode[2]);
				double y = Double.parseDouble(stringNode[3]);
				int maxCapacity = (int)Double.parseDouble(stringNode[4]);
				int initialOccupancy = (int)Double.parseDouble(stringNode[5]);
				String nodeTypeC = stringNode[6];
				int nodeType = 2;
				if(nodeTypeC.equals("S"))
					nodeType = Node.SOURCE;
				else if(nodeTypeC.equals("D"))
					nodeType = Node.DESTINATION;
				else
					nodeType = Node.NORMAL;
				ccrp.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				basicCCRP2.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				ccrpPlusPlus.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				//shelterBasedCCRP.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				oneSourceOnePath.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
				prefToLongerPaths.addNodeToGraph(nodeId, nodeName, x, y, maxCapacity, initialOccupancy, nodeType);
			}
			csvFile = edgeFile;
			br.close();
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				count++;
				
				String[] stringEdge = line.split(cvsSplitBy);
				int edgeID = Integer.parseInt(stringEdge[0]);
				String edgeName = stringEdge[1];
				String sourceName = stringEdge[2];
				String targetName = stringEdge[3];
				int travelTime = (int)Double.parseDouble(stringEdge[4]);
				int maxCapacity = (int)Double.parseDouble(stringEdge[5]);
				sum+=maxCapacity;
				ccrp.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				basicCCRP2.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				ccrpPlusPlus.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				//shelterBasedCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				oneSourceOnePath.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				prefToLongerPaths.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
			}
		} 
		catch (FileNotFoundException e) 
		{
			e.printStackTrace();
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		} 
		finally 
		{
			if (br != null) {
				try 
				{
					br.close();
				} 
				catch (IOException e) 
				{
					e.printStackTrace();
				}
			}
		}
	}
	
	public static void readNodeAndEdgeFiles(String nodeFile, String edgeFile, int overload)
	{
		String csvFile = nodeFile;
		BufferedReader br = null;
		String line = "";
		String cvsSplitBy = ",";
		try 
		{
	 
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				//System.out.println(line);
				String[] stringNode = line.split(cvsSplitBy);
				String nodeId = stringNode[0];
				String nodeName = stringNode[1];
				int maxCapacity = (int)Double.parseDouble(stringNode[2]);
				int initialOccupancy = (int)Double.parseDouble(stringNode[3]);
				String nodeTypeC = stringNode[4];
				int nodeType = 2;
				if(nodeTypeC.equals("S"))
					nodeType = Node.SOURCE;
				else if(nodeTypeC.equals("D"))
					nodeType = Node.DESTINATION;
				else
					nodeType = Node.NORMAL;
				ccrp.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				kiaCCRP.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				basicCCRP.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				ccrpPlusPlus.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				//shelterBasedCCRP.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				oneSourceOnePath.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
				prefToLongerPaths.addNodeToGraph(nodeId, nodeName, maxCapacity, initialOccupancy, nodeType);
			}
			csvFile = edgeFile;
			br.close();
			br = new BufferedReader(new FileReader(csvFile));
			while ((line = br.readLine()) != null) 
			{
				count++;
				//System.out.println(line);
				String[] stringEdge = line.split(cvsSplitBy);
				int edgeID = Integer.parseInt(stringEdge[0]);
				String edgeName = stringEdge[1];
				String sourceName = stringEdge[2];
				String targetName = stringEdge[3];
				int travelTime = (int)Double.parseDouble(stringEdge[4]);
				int maxCapacity = (int)Double.parseDouble(stringEdge[5]);
				sum+=maxCapacity;
				ccrp.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				kiaCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				basicCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				ccrpPlusPlus.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				//shelterBasedCCRP.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				oneSourceOnePath.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				prefToLongerPaths.addEdgeToGraph(edgeID, edgeName, sourceName, targetName, maxCapacity, travelTime);
				
			}
		} 
		catch (FileNotFoundException e) 
		{
			e.printStackTrace();
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		} 
		finally 
		{
			if (br != null) {
				try 
				{
					br.close();
				} 
				catch (IOException e) 
				{
					e.printStackTrace();
				}
			}
		}
	}
	public static void main(String args[])
	{
		ccrp = new CCRPWithReuse1();
		kiaCCRP = new KiaCCRP();
		basicCCRP = new BasicCCRP1();
		basicCCRP2 = new BasicCCRP1();
		ccrpPlusPlus = new CCRP_PlusPlus1();
		shelterBasedCCRP = new ShelterBasedCCRP();
		oneSourceOnePath = new OneSourceOnePath();
		prefToLongerPaths = new PreferenceToLongerPaths();
		String nodeFile = "ProjectFiles/NodeFiles/node42.csv";
		String edgeFile = "ProjectFiles/EdgeFiles/edge42.csv";
		//System.out.println("ABC");
		readNodeAndEdgeFiles(nodeFile, edgeFile, 1);
		
		//ccrpPlusPlus.ccrpPlusPlus();
		System.out.println("average Edge Cap :" + Math.round(1.0*sum/count));
		//shelterBasedCCRP.modifiedCCRPEvacuationPlanner();
		//System.out.println("BCD");
		System.out.println("Modified CCRP");
		long startTime1 = System.currentTimeMillis();
		ccrp.runNo=1;
		ccrp.modifiedCCRPEvacuationPlanner();
		long endTime1 = System.currentTimeMillis();
		System.out.println("Running time :" + (endTime1-startTime1) );
		
		System.out.println("\n\nRun2");
		String nodeFile2 = "/Users/MLGupta/Documents/node42.csv";
		changeNodeCap(nodeFile2);
		ccrp.runNo=2;
		ccrp.clearNodeEdgeData();
		ccrp.modifiedCCRPEvacuationPlanner();
		
		System.out.println("\n\n");
		ccrp.findResults();
		//ccrp.displayNodeEdgeStats();
		System.out.println("***********");
		System.out.println("Basic CCRP");
		long startTime = System.currentTimeMillis();
		//basicCCRP.CCRPEvacuationPlanner();
		nodeFile = "/Users/MLGupta/Documents/node_NoPplInHall.csv";
		edgeFile = "/Users/MLGupta/Documents/edge.csv";
		//System.out.println("ABC");
		//readNodeAndEdgeFiles(nodeFile, edgeFile,"a");
		//basicCCRP2.setRunNo(1);
		//basicCCRP2.clearData();
		//basicCCRP2.CCRPEvacuationPlanner();

		long endTime = System.currentTimeMillis();
		System.out.println("Running time :" + (endTime-startTime) );
		//basicCCRP2.displayNodeEdgeStats();
		System.out.println("***********");
		System.out.println("CCRP Plus Plus");
		startTime1 = System.currentTimeMillis();
		//ccrpPlusPlus.ccrpPlusPlus();
		endTime1 = System.currentTimeMillis();
		System.out.println("Running time :" + (endTime1-startTime1) );
		startTime1 = System.currentTimeMillis();
		//ccrpPlusPlus.displayNodeEdgeStats();
		//kiaCCRP.modifiedCCRPEvacuationPlanner();
		endTime1 = System.currentTimeMillis();
		System.out.println("Running time :" + (endTime1-startTime1) );
	}
}
