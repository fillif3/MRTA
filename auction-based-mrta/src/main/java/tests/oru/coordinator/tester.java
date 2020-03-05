package tests.oru.coordinator;

import net.sourceforge.jFuzzyLogic.FIS;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

//import coordinationNew.TrajectoryEnvelopeCoordinatorSimulationNew;
import java.util.ArrayList;
import java.util.List;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.PathEditor2;

import java.io.BufferedReader; 
import java.io.IOException; 
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.Random;
import java.lang.String;

public class tester {
	
	/*public static double[][] fuzzyBid(double[][] distances,double[][] cs)
	{
		cs = TestExternalMotionPlanner1.normalize(cs);
		distances = TestExternalMotionPlanner1.normalize(distances);
		double[][] bid = new double[distances.length][distances[0].length];
        String fileName = "/home/filip/Pulpit/projekt/auction-based-mrta/src/main/java/tests/oru/coordinator/tipper.fcl"; //add file
        ///tipper.fcl
        //final File folder = new File(fileName);
        //listFilesForFolder(folder);
        FIS fis = FIS.load(fileName,true);
		for (int i=0;i<distances.length;i++)
		{
			for (int j=0;j<distances[0].length;j++)
			{
				fis.setVariable("distance", distances[i][j]);
		        fis.setVariable("cs", cs[i][j]);
		        fis.evaluate();
		        bid[i][j] = fis.getVariable("bid").getValue();
			}
		}
		return bid;
	}
	
	public static double[][] randomDist(int number)
	{
		double[][] values = new double[number][number];
		Random rnd = new Random();
		for (int i=0;i<number;i++)
		{
			for (int j=0;j<number;j++)
			{
				values[i][j] = rnd.nextDouble();
			}
		}
		
		return values;
	}
	
	public static double[][] randomCs(int number)
	{
		double[][] values = new double[number][number];
		Random rnd = new Random();
		for (int i=0;i<number;i++)
		{
			for (int j=0;j<number;j++)
			{
				values[i][j] = Math.floor(rnd.nextDouble()*10);
			}
		}
		
		return values;
	}
	
	private static boolean[][] check(double[][] distances,double[][] cs) {
		
		boolean[][] takingIntoAccount = new boolean[cs.length][cs[0].length];
		takingIntoAccount = checkDiagonal(takingIntoAccount);
		takingIntoAccount = checkDangerous(takingIntoAccount,cs);
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

	private static boolean[][] checkDangerous(boolean[][] takingIntoAccount, double[][] cs) {
		int[] index = TestExternalMotionPlanner1.FindBiggest(cs);
		return crossOut(takingIntoAccount,index);
	}

	private static boolean[][] checkDiagonal(boolean[][] takingIntoAccount) {
		for (int i=0;i<takingIntoAccount.length;i++)
		{
			takingIntoAccount[i][i] = true;
		}
		return takingIntoAccount;
	}

    public static boolean[][] crossOut(boolean[][] votes,int[] out)
    {
    	boolean[][] copyOfVotes = Arrays.stream(votes).map(boolean[]::clone).toArray(boolean[][]::new);


		copyOfVotes = zeroColumn(copyOfVotes,out[1]);
		copyOfVotes = zeroRow(copyOfVotes,out[0]);

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
    
	private static double[][] sortAccordingToPrediction(int[] prediction, double[][] distances) {
		double[][] newDisctances = new double[prediction.length][prediction.length];
		for (int i=0;i<prediction.length;i++)
		{
			for (int j=0;j<prediction.length;j++)
			{
				newDisctances[i][j] = distances[prediction[i]][j];
			}
		}
		return newDisctances;
	}
	
	private static int checkComputations(boolean[][] takingIntoAccount) {
		int k =1;
		
		for (int i=0;i<(takingIntoAccount.length-1);i++)
		{
			for (int j=(i+1);j<takingIntoAccount.length;j++)
			{
				if (takingIntoAccount[i][j] || takingIntoAccount[j][i] )
				{
					k++;
				}
				
			}
			
		}
		
		return k;
		
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	public static void main(String[] args) throws InterruptedException, IOException {
        
        int[] min = new int[98];
        int[] max = new int[98];
        int mi = 1000000;
        int ma = 0;
        double[] ave = new double[98];
        int k =0;
		for (int i=3;i<101;i++)
		{
			for (int j=0;j<10;j++)
			{
		        
				double[][] distances = randomDist(i);
				double[][] cs = randomCs(i);
				int[] prediction = TestExternalMotionPlanner1.auction(distances);
				distances = sortAccordingToPrediction(prediction,distances);
				//if (TestExternalMotionPlanner1.getMax(cs)==0) return prediction;
				boolean[][] takingIntoAccount= check(distances,cs);
				//double[][] bid = fuzzyBid(distances,cs);
				int computations = checkComputations(takingIntoAccount);
				//int[] alloc = TestExternalMotionPlanner1.auction(bid);
				System.out.print(i);
				k =+computations;
				if (computations>ma) ma = computations;
				if (computations<mi) mi = computations;
			}
			min[i-3] = mi;
			max[i-3] = ma;
			ave[i-3] = Double.valueOf(k)/10;
	        mi = 1000000;
	        ma = 0;
	        k=0;
	        System.out.print(i);
				
		}
		
		PrintWriter writer = new PrintWriter("the-file-name.txt", "UTF-8");
	    String mins = new String("min = [");
	    for (int i=0;i<98;i++)
	    {
	    	mins = mins.concat(Double.toString(min[i]));
	    	mins = mins.concat(" ");
	    }
	    mins = mins.concat("]");
	    writer.println(mins);
	    String maxs = new String("max = [");
	    for (int i=0;i<98;i++)
	    {
	    	maxs = maxs.concat(String.valueOf(max[i]));
	    	maxs = maxs.concat(" ");
	    }
	    maxs = maxs.concat("]");
	    writer.println(maxs);
	    String aves = new String("ave = [");
	    for (int i=0;i<98;i++)
	    {
	    	aves = aves.concat(String.valueOf(ave[i]));
	    	aves = aves.concat(" ");
	    }
	    aves = aves.concat("]");
	    writer.println(aves);
	    writer.close();
	    //System.out.println(s);
      
     // Reading data using readLine 
     /*   int number = Integer.parseInt(reader.readLine());
      
		double[][] distances = randomDist(number);
		double[][] cs = randomCs(number);
		int[] prediction = TestExternalMotionPlanner1.auction(distances);
		distances = sortAccordingToPrediction(prediction,distances);
		//if (TestExternalMotionPlanner1.getMax(cs)==0) return prediction;
		boolean[][] takingIntoAccount= check(distances,cs);
		//double[][] bid = fuzzyBid(distances,cs);
		int computations = checkComputations(takingIntoAccount);
		//int[] alloc = TestExternalMotionPlanner1.auction(bid);
		System.out.print(computations);
	}

*/


}
