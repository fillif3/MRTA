package tests.oru.coordinator;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import net.sourceforge.jFuzzyLogic.FIS;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class Allocation {
	
	 public static int[] FindSmallest (double [][] arr1) {
	        int[] index = {0,0};
	        double min = arr1[index[0]][index[1]];

	        for (int i=0; i<arr1.length; i++) {
	            for (int j=0; j<arr1[1].length; j++)
	            {
	                if (arr1[i][j] < min) {
	                    min = arr1[i][j];
	                    index[0] = i;
	                    index[1] = j;
	                }
	            }
	        }
	        return index;
	    }
	    
	    public static int[] FindBiggest (double[][] arr1) {
	        int[] index = {0,0};
	        double max = arr1[index[0]][index[1]];

	        for (int i=0; i<arr1.length; i++) {
	            for (int j=0; j<arr1[1].length; j++)
	            {
	                if (arr1[i][j] > max) {
	                    max = arr1[i][j];
	                    index[0] = i;
	                    index[1] = j;
	                }
	            }
	        }
	        return index;
	    }
	    
	    public static int FindBiggest (double[] arr1) {
	        int index = 0;
	        double max = arr1[0];

	        for (int i=0; i<arr1.length; i++) {

	                if (arr1[i] > max) {
	                    max = arr1[i];
	                    index = i;

	                }
	            
	        }
	        return index;
	    }
	    
	    public static int FindExtacly (int[] arr1,int value) {


	        for (int i=0; i<arr1.length; i++) {
	            
	                if (arr1[i] == value) {
	                    return i;
	                }
	            
	        }
	        return -1;
	    }
	    
	    public static int FindBiggestCol (double[][] arr1,int col) {
	        int index = 0;
	        double max = arr1[index][col];

	            for (int j=0; j<arr1.length; j++)
	            {
	                if (arr1[j][col] > max) {
	                    max = arr1[col][j];
	                    index= j;
	                }
	            }
	        
	        return index;
	    }
	    
	    public static int FindBiggestRow (double[][] arr1,int row) {
	        int index = 0;
	        double max = arr1[row][index];

	            for (int j=0; j<arr1[0].length; j++)
	            {
	                if (arr1[row][j] > max) {
	                    max = arr1[row][j];
	                    index= j;
	                }
	            }
	        
	        return index;
	    }
	    
	    public static int contains(final int[] array, final int v) {

	        int result = 0;

	        for(int i : array){
	            if(i == v){
	                result++;
	            }
	        }

	        return result;
	    }
	    
	    public static int contains(final boolean[] array, final boolean v) {

	        int result = 0;

	        for(boolean i : array){
	            if(i == v){
	                result++;
	            }
	        }

	        return result;
	    }
	    
	    public static double[][] crossOut(double[][] votes,int[] out)
	    {
	    	double[][] copyOfVotes = Arrays.stream(votes).map(double[]::clone).toArray(double[][]::new);
	    	for (int i=0;i<out.length;i++)
	    	{
	    		if (out[i]!=-1)
	    		{
	    			copyOfVotes = zeroColumn(copyOfVotes,out[i]);
	    			copyOfVotes = zeroRow(copyOfVotes,i);
	    		}
	    	}
	    	return copyOfVotes;
	    }
	    
	    public static double[][] zeroColumn(double[][] votes,int index)
	    {
	    	double[][] copyOfVotes = Arrays.stream(votes).map(double[]::clone).toArray(double[][]::new);
	    	for (int i=0;i<votes.length;i++) copyOfVotes[i][index]=0;
	    	return copyOfVotes;
	    }
	    
	    public static double[][] zeroRow(double[][] votes,int index)
	    {
	    	double[][] copyOfVotes = Arrays.stream(votes).map(double[]::clone).toArray(double[][]::new);
	    	for (int i=0;i<votes[0].length;i++) copyOfVotes[index][i]=0;
	    	return copyOfVotes;
	    }

	    public static int nonZeroElements(double[][] votes)
	    {
	        int result = 0;

	        for(int i=0;i<votes.length;i++) {
	        	for(int j=0;j<votes.length;j++)
	        	{
	        		if( votes[i][j]!= 0){
	                    result++;
	                }
	        	}
	            
	        }

	        return result;
	    }
	    
	    public static int nonZeroElementsCol(double[][] votes,int index)
	    {
	        int result = 0;

	        for(int i=0;i<votes.length;i++) {
	            if( votes[i][index]!= 0){
	                result++;
	            }
	        }

	        return result;
	    }
	    
	    public static int nonZeroElementsRow(double[][] votes,int index)
	    {
	        int result = 0;

	        for(int i=0;i<votes[0].length;i++) {
	            if( votes[index][i]!= 0){
	                result++;
	            }
	        }

	        return result;
	    }
		
	    public static int[] auction (double[][] votes)
	    {
	        int tl = votes[0].length;
	        int index=0;
	 
	        //int amountOfAllocations = 0;
	        int[] out = new int[tl];
	        for (int j=0;j<tl;j++)
	        {
	            out[j]=-1;
	        }
	        boolean flag;
	        int[] i = new int[2];
	        while(true)
	        {
	        	flag = false;
	        	//double[][] copyOfVotes = Arrays.;
	        	double[][] copyOfVotes = crossOut(votes,out);
	        	while (true)
	        	{
		            i=FindBiggest(copyOfVotes);
		            if (nonZeroElements(copyOfVotes)>1) copyOfVotes[i[0]][i[1]] = 0;
		            if (nonZeroElementsCol(copyOfVotes,i[1])==1)
		            {
		            	flag=true;
		            	index = FindBiggestCol(copyOfVotes,i[1]);
		            	out[index] = i[1];
		            	
		            }
		            if (nonZeroElementsRow(copyOfVotes,i[0])==1)
		            {
		            	flag=true;
		            	index = FindBiggestRow(copyOfVotes,i[0]);
		            	out[i[0]] = index;
		            }
		            
		            if(flag) break;
		            
	        	}
	        	if (contains(out,-1)==0) break;

	        }
	        return out;
	    }
	    
	    public static void listFilesForFolder(final File folder) {
	        for (final File fileEntry : folder.listFiles()) {
	            if (fileEntry.isDirectory()) {
	                listFilesForFolder(fileEntry);
	            } else {
	                System.out.println(fileEntry.getName());
	            }
	        }
	    }
		
		public static int[] allocation(Pose[] start,Pose[] end,ReedsSheppCarPlanner rsp,Coordinate ... coordinates)
		{
			List<List<PoseSteering[]>> paths = checkPaths(start,end,rsp);

			
			double[][] distances = distance(paths);
			int[] prediction = auction(distances);
			
			int[] alloc = correction(start,end,rsp,prediction,distances,paths);
			return alloc;
		}
		
		public static int[] Eucalidian(Pose[] start,Pose[] end)
		{


			
			double[][] distances = new double[start.length][end.length];
	        for(int i=0;i<start.length;i++) {
	        	for(int j=0;j<end.length;j++)
	        	{
	        		distances[i][j]=distancePose(start[i].getX(),start[i].getY(),end[j].getX(),end[j].getY());
	                
	        	}
	            
	        }

			
			int[] alloc = auction(distances);
			
			return alloc;
		}
		
		
		
		/*public static double[] computeCS(Pose[] robots,Pose[] end,ReedsSheppCarPlanner rsp,int[] prediction)
		{
			List<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();
			for (int i=0;i<robots.length;i++)
			{
				rsp.setStart(robots[i]);
				rsp.setGoals(end[prediction[i]]);
				rsp.plan();	
				PoseSteering[] pss1 = rsp.getPath();
				paths.add(pss1);
			}
			
			for(int i=0;i<robots.length;i++)
			{
				Missions.enqueueMission(new Mission(i+1, paths.get(i)));
			}
		}*/
		
		/*public static double[] computeCS(Pose[] robots,Pose[] end,ReedsSheppCarPlanner rsp,int[] prediction,int r1,int r2)
		{
			List<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();
			for (int i=0;i<robots.length;i++)
			{
				rsp.setStart(robots[i]);
				rsp.setGoals(end[prediction[i]]);
				rsp.plan();	
				// Path is always null, I do not know why
				PoseSteering[] pss1 = rsp.getPath();
				paths.add(pss1);
			}
			
			for(int i=0;i<robots.length;i++)
			{
				Missions.enqueueMission(new Mission(i+1, paths.get(i)));
			}
			
			
		}*/
		
		public static double[][] criticalSection(Pose[] robots,Pose[] end,ReedsSheppCarPlanner rsp,int[] prediction)
		{
			double[][] cs = new double[robots.length][robots.length];
			
			//double[] out = computeCS(robots,end,rsp,prediction);
			double[] out =new double[9];
			for(int i=0;i<robots.length;i++)
			{
				cs[i][i] = out[i];
			}
			
			int help;
			
			for(int i=1;i<robots.length;i++)
			{
				 for(int j=(i+1);j<robots.length;j++)
				 {
					 help = prediction[i];
					 prediction[i]=prediction[j];
					 prediction[j] = help;
					 //out = computeCS(robots,end,rsp,prediction,i,j);
					 
					 cs[i][j] = out[0];
					 cs[j][i]= out[1];
					 help = prediction[i];
					 prediction[i]=prediction[j];
					 prediction[j] = help;
					 
				 }
			}
			
			return cs;
		}
		
		public static int[] correction(Pose[] start,Pose[] end,ReedsSheppCarPlanner rsp,int[] prediction,double[][] distances,List<List<PoseSteering[]>> paths)
		{
			
			//double[][] cs = criticalSections(start,end,rsp,prediction); //need to finish
			double[][] polygon = {{-1.0,0.5},{1.0,0.5},{1.0,-0.5},{-1.0,-0.5}};
			List<double[][]> polygons = new ArrayList<double[][]>();
			polygons.add(polygon);
			polygons.add(polygon);
			double[] cs = collision.getCs(paths, polygons, prediction);
			if (getMax(cs)==0) return prediction;
			boolean[][] takingIntoAccount= check(distances,cs,prediction);
			double[][] bid = fuzzyBid(distances,polygons,takingIntoAccount,prediction,paths,cs);
			int[] alloc = auction(bid);
			return alloc;
		}	
		
		private static boolean[][] check(double[][] distances, double[] cs,int[] prediction) {
			boolean[][] takingIntoAccount = new boolean[cs.length][cs.length];
			takingIntoAccount = checkDiagonal(takingIntoAccount,prediction);
			takingIntoAccount = checkDangerous(takingIntoAccount,cs,prediction);
			takingIntoAccount = checkLower(takingIntoAccount,distances);
			return takingIntoAccount;
		}

		private static boolean[][] checkLower(boolean[][] takingIntoAccount, double[][] distances) {
			
			double min = Double.MAX_VALUE;
			int index = 0;
			for (int i=0;i<takingIntoAccount.length;i++)
			{
				for (int j=0;j<takingIntoAccount.length;j++)
				{
					if (takingIntoAccount[i][j] == false)
					{
						if (min>distances[i][j])
						{
							min = distances[i][j];
							index = j;
						}
					}
					
				}
				
				takingIntoAccount[i][index] = true;
			    min = Double.MAX_VALUE;
				index = 0;
			}
			return takingIntoAccount;
		}

		
		private static boolean[][] checkDangerous(boolean[][] takingIntoAccount, double[] cs,int[] prediction) {
			int index = FindBiggest(cs);
			return crossOut(takingIntoAccount,index,prediction);
		}

	    public static boolean[][] crossOut(boolean[][] votes,int out,int[] prediction)
	    {
	    	boolean[][] copyOfVotes = Arrays.stream(votes).map(boolean[]::clone).toArray(boolean[][]::new);


			copyOfVotes = zeroColumn(copyOfVotes,prediction[out]);
			copyOfVotes = zeroRow(copyOfVotes,out);

	    	return copyOfVotes;
	    }

	    public static boolean[][] zeroColumn(boolean[][] votes,int index)
	    {
	    	boolean[][] copyOfVotes = Arrays.stream(votes).map(boolean[]::clone).toArray(boolean[][]::new);
	    	for (int i=0;i<votes.length;i++) copyOfVotes[i][index]=true;
	    	return copyOfVotes;
	    }
	    
	    public static boolean[][] zeroRow(boolean[][] votes,int index)
	    {
	    	boolean[][] copyOfVotes = Arrays.stream(votes).map(boolean[]::clone).toArray(boolean[][]::new);
	    	for (int i=0;i<votes[0].length;i++) copyOfVotes[index][i]=true;
	    	return copyOfVotes;
	    }

		private static boolean[][] checkDiagonal(boolean[][] takingIntoAccount, int[] prediction) {
			for (int i=0;i<takingIntoAccount.length;i++)
			{
				takingIntoAccount[i][prediction[i]] = true;
			}
			return takingIntoAccount;
		}

		public static double[][] fuzzyBid(double[][] distances,List<double[][]> polygons,boolean[][] takingIntoAccount,int[] prediction,List<List<PoseSteering[]>> paths,double[] cs)
		{
			double maxCS = getMax(cs);
			cs = normalize(cs);
			distances = normalize(distances);
			
			int leng = distances.length;
			double[][] bid = new double[leng][leng];
	        String fileName = "src/main/java/tests/oru/coordinator/tipper.fcl"; //add file
	        ///tipper.fcl   criticalSections
	        //final File folder = new File(fileName);
	        //listFilesForFolder(folder);
	        FIS fis = FIS.load(fileName,true);
			for (int i=0;i<leng;i++)
			{
				fis.setVariable("distance", distances[i][prediction[i]]);
		        fis.setVariable("cs", cs[i]);
		        fis.evaluate();
		        bid[i][prediction[i]] = fis.getVariable("bid").getValue();
		        takingIntoAccount[i][prediction[i]]=false;
			}
	        
	        
			for (int i=0;i<leng;i++)
			{
				for (int j=0;j<leng;j++)
				{
					if(takingIntoAccount[i][j])
					{
						int newJ = FindExtacly(prediction,j);
						int help = prediction[i];
						prediction[i] = prediction[j];
						prediction[j] = help;
						int[] index = {i,j};
						cs = collision.getCs(paths,polygons,prediction,index);
						cs = normalize(cs,maxCS);
						fis.setVariable("distance", distances[i][j]);
				        fis.setVariable("cs", cs[0]);
				        fis.evaluate();
				        bid[i][j] = fis.getVariable("bid").getValue();
				        takingIntoAccount[i][j]=false;
				        help = prediction[j];
						prediction[j] = prediction[i];
						prediction[i] = help;
					}
				}
			}
			
			for (int i=0;i<leng;i++)
			{
				for (int j=0;j<leng;j++)
				{
					if (bid[i][j]==0) bid[i][j] = 100;
				}
			}
			return bid;
		}
		
		
		public static double RowSum(double[][] distances,int i)
		{
			double result=0;
			for(int k=0;k<distances.length;k++)
			{
				result =+distances[i][k];
			}
			return result;
		}
		
		public static double ColSum(double[][] distances,int j)
		{
			double result=0;
			for(int k=0;k<distances.length;k++)
			{
				result =+distances[k][j];
			}
			return result;
		}
		
		public static double[][] creatingWeights(double[][] distances)
		{
			double maximum = getMax(distances);
			double[][] weights = new double[distances.length][distances.length];
			for(int i=0;i < distances.length;i++){ 
				for(int j=0;j < distances[0].length;j++){ 

				    weights[i][j] = (maximum - distances[i][j])/maximum;
				    
				}
			} 
			for(int i=0;i < distances.length;i++){ 
				for(int j=0;j < distances[0].length;j++){ 

				    weights[i][j] = weights[i][j]/RowSum(weights,i)+weights[i][j]/ColSum(weights,j);
				    
				}
			} 
			return weights;
		}
		
		public static double getMax(double[][] inputArray){ 
			double maxValue = inputArray[0][0]; 
			for(int i=0;i < inputArray.length;i++){ 
				for(int j=0;j < inputArray[0].length;j++){ 
					if(inputArray[i][j] > maxValue){ 
						maxValue = inputArray[i][j]; 
				    } 
				}
			 } 
			 return maxValue; 
		}
		
		public static double getMax(double[] inputArray){ 
			double maxValue = inputArray[0]; 
			for(int i=0;i < inputArray.length;i++){ 

					if(inputArray[i] > maxValue){ 
						maxValue = inputArray[i]; 
				    } 
				
			 } 
			 return maxValue; 
		}
		
		public static double[][] normalize(double[][] inputArray)
		{
			double max = getMax(inputArray);
			for (int i=0;i<inputArray.length;i++)
			{
				for (int j=0;j<inputArray[0].length;j++)
				{
					inputArray[i][j] = inputArray[i][j]/max;
				}
			}
			return inputArray;
		}
		
		public static double[] normalize(double[] inputArray)
		{
			double max = getMax(inputArray);
			for (int i=0;i<inputArray.length;i++)
			{

					inputArray[i] = inputArray[i]/max;
				
			}
			return inputArray;
		}
		
		public static double[][] normalize(double[][] inputArray,double max)
		{
			for (int i=0;i<inputArray.length;i++)
			{
				for (int j=0;j<inputArray[0].length;j++)
				{
					inputArray[i][j] = inputArray[i][j]/max;
				}
			}
			return inputArray;
		}
		
		public static double[] normalize(double[] inputArray,double max)
		{
			for (int i=0;i<inputArray.length;i++)
			{

				inputArray[i] = inputArray[i]/max;
				if (inputArray[i]>1) inputArray[i]=1;
				
			}
			return inputArray;
		}
		
		public static double[][] distance(List<List<PoseSteering[]>> paths)
		{
			int l = paths.size();
			double[][] dist = new double[l][l];
			for (int i=0;i<l;i++)
			{
				for (int j=0;j<l;j++)
				{
					//rsp.clearObstacles();
					//Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
					/*for (int k=0;k<l;k++)
					{
						if (k!=i) 
						{
							rsp.addObstacles(fpGeom, start[i]);
						}
					}
					for (int k=0;k<l;k++)
					{
						if (k!=j) 
						{
							rsp.addObstacles(fpGeom, end[j]);
						}
					}*/
					//if (!rsp.plan()) throw new Error ("No path between " + start[i] + " and " + end[j]);  
					if (paths.get(i).get(j) == null) dist[i][j] = Double.MAX_VALUE;
					else dist[i][j]= distancePath((PoseSteering[]) paths.get(i).get(j));
					
					// pss1[i].pose.x
				}
			}
			
			return dist;
		}

		public static List<List<PoseSteering[]>> checkPaths(Pose[] start,Pose[] end,ReedsSheppCarPlanner rsp)
		{
			int l = start.length;
			List<List<PoseSteering[]>> table = new ArrayList<List<PoseSteering[]>>();
			
			//List<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();
			
			//double[][] dist = new double[l][l];
			for (int i=0;i<l;i++)
			{
				List<PoseSteering[]> helpList = new ArrayList<PoseSteering[]>();
				for (int j=0;j<l;j++)
				{
					//rsp.clearObstacles();
					rsp.setStart(start[i]);
					rsp.setGoals(end[j]);
					//Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
					/*for (int k=0;k<l;k++)
					{
						if (k!=i) 
						{
							rsp.addObstacles(fpGeom, start[i]);
						}
					}
					for (int k=0;k<l;k++)
					{
						if (k!=j) 
						{
							rsp.addObstacles(fpGeom, end[j]);
						}
					}*/
					//if (!rsp.plan()) throw new Error ("No path between " + start[i] + " and " + end[j]);  
					rsp.plan();	

					// Path is always null, I do not know why
					PoseSteering[] pss1 = rsp.getPath();
					helpList.add(pss1);
					//if (pss1 == null) dist[i][j] = Double.MAX_VALUE;
					//else dist[i][j]= distancePath(pss1);
					
					// pss1[i].pose.x
				}
				table.add(helpList);
				PoseSteering[] pss2 = table.get(0).get(0);
				@SuppressWarnings("unused")
				int k =1;
			}
			
			return table;
		}
		
		public static double distancePath(PoseSteering[] path)
		{
			int l = path.length;
			double dist = 0;
			double k;
			for (int i=0;i<(l-1);i++)
			{
				k=distancePose(path[i].getX(),path[i].getY(),path[i+1].getX(),path[i+1].getY());
				dist += k;
			}
			
			return dist;
		}
		public static double distancePose(double x1, double y1,double x2, double y2)
		{
			return Math.sqrt(Math.pow((x1-x2),2)+Math.pow((y1-y2),2));
		}
		
	

}