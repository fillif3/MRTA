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

public class collision {
	

	// A recursive function to replace previous color 'prevC' at '(x, y)'  
	// and all surrounding pixels of (x, y) with new color 'newC' and 
	static void floodFillUtil(int screen[][], int x, int y,  
	                                    int prevC, int newC,int M,int N) 
	{ 
	    // Base cases 
	    if (x < 0 || x >= M || y < 0 || y >= N) 
	        return; 
	    if (screen[x][y] != prevC) 
	        return; 
	  
	    // Replace the color at (x, y) 
	    screen[x][y] = newC; 
	  
	    // Recur for north, east, south and west 
	    floodFillUtil(screen, x+1, y, prevC, newC,M,N); 
	    floodFillUtil(screen, x-1, y, prevC, newC,M,N); 
	    floodFillUtil(screen, x, y+1, prevC, newC,M,N); 
	    floodFillUtil(screen, x, y-1, prevC, newC,M,N); 
	} 
	  
	// It mainly finds the previous color on (x, y) and 
	// calls floodFillUtil() 
	static void floodFill(int screen[][], int x, int y, int newC) 
	{ 
	    int prevC = screen[x][y]; 
	    int M = screen.length;
	    int N = screen[0].length;
	    floodFillUtil(screen, x, y, prevC, newC,M,N); 
	} 
	
	public static double[] getCs(List<List<PoseSteering[]>> paths,List<double[][]> polygons,int[] prediction)
	{
		int l = polygons.size();
		double[] safeDistances = new double[l];
		double[] dangerousDistances = new double[l];
		for (int i=0;i<l;i++)
		{
			safeDistances[i] = safeDist(polygons.get(i));
			dangerousDistances[i] = dangerousDist(polygons.get(i));
		}
		
		
		
		double[] cs =new double[l];
		
		for (int i=0;i<(l-1);i++)
		{
			for (int j=(i+1);j<l;j++)
			{
				double[] help = collisionMap(paths.get(i).get(prediction[i]),polygons.get(i),paths.get(j).get(prediction[j]),polygons.get(j), safeDistances[i],dangerousDistances[i],safeDistances[j],dangerousDistances[j]);
				
				cs[i]= cs[i]+help[0];
				cs[j]= cs[j]+help[1];
			
			}
		}
		
		return cs;
	}
	
	public static double[] getCs(List<List<PoseSteering[]>> paths,List<double[][]> polygons,int[] prediction,int[] index)
	{
		int l = polygons.size();
		int indexl = index.length;
		double[] safeDistances = new double[l];
		double[] dangerousDistances = new double[l];
		for (int i=0;i<l;i++)
		{
			safeDistances[i] = safeDist(polygons.get(i));
			dangerousDistances[i] = dangerousDist(polygons.get(i));
		}
		
		double[]cs =new double[l];
		
		for (int i=0;i<(indexl-1);i++)
		{
			for (int j=(i+1);j<indexl;j++)
			{
				double[] help  = collisionMap(paths.get(i).get(prediction[i]),polygons.get(i),paths.get(j).get(prediction[j]),polygons.get(j), safeDistances[i],dangerousDistances[i],safeDistances[j],dangerousDistances[j]);
				cs[i]= cs[i]+help[0];
				cs[j]= cs[j]+help[1];
			
			}
		}
		
		for (int i=0;i<(indexl-1);i++)
		{
			for (int j=(i+1);j<l;j++)
			{
				if (Allocation.contains(index, j)==0)
				{
					double[] help = collisionMap(paths.get(i).get(prediction[i]),polygons.get(i),paths.get(j).get(prediction[j]),polygons.get(j), safeDistances[i],dangerousDistances[i],safeDistances[j],dangerousDistances[j],true);
					cs[i]= cs[i]+help[0];
				}
			
			}
		}
		
		return cs;
	}
	
	
	public static int getCsFromMap(int[][] collisionMap)
	{
		int track =0;
		
		for (int i = 0;i<(collisionMap.length-1);i++)
		{
			for (int j = 0;j<(collisionMap[0].length-1);j++)
			{
				if (collisionMap[i][j]==1)
				{
					track++;
					floodFill(collisionMap, i, j, 2);
				}
				
			}

		}

		return track;
		
	}
	

	public static double[] collisionMap(PoseSteering[] path1, double[][] polygon1,PoseSteering[] path2, double[][] polygon2, double safe1,double dangerous1,double safe2,double dangerous2)
	{
		int l1 = ((path1.length-1)/10)+1;
		int l2 = ((path2.length-1)/10)+1;

		int len;
		
		if (l1<l2)
		{
			len=l2;
		}
		else
		{
			len=l1;
		}
		
		int[][] map = new int[2][len];
		
		for (int i = 0;i<(path1.length-1);i++)
		{
			double rot1 = direction(path1[i].getPose(),path1[i+1].getPose());
			double[][] nodes1 = place(polygon1,path1[i].getPose().getX(),path1[i].getPose().getY(),rot1);
			for (int j = 0;j<(path2.length-1);j++)
			{
				if ((map[0][i/10]==0) ||(map[1][j/10]==0) )
				{
					double rot2 = direction(path2[j].getPose(),path2[j+1].getPose());
					double[][] nodes2 = place(polygon2,path2[j].getPose().getX(),path2[j].getPose().getY(),rot2);
					if (detectCollision(nodes1,path1[i].getPose(),safe1,dangerous1,nodes2,path2[j].getPose(),safe2,dangerous2))
					{
						map[0][i/10] = 1;
						map[1][j/10] = 1;
					}
					
				}
			}

		}
		
		double[] cs = new double[2];
		cs[0] = ((double)Allocation.contains(map[0], 1))/l1;
		cs[1] = ((double)Allocation.contains(map[1], 1))/l2;
		
		return cs;
	}	
	public static double[] collisionMap(PoseSteering[] path1, double[][] polygon1,PoseSteering[] path2, double[][] polygon2, double safe1,double dangerous1,double safe2,double dangerous2,boolean flag)
	{
		int l1 = ((path1.length-1)/10)+1;

		
		int[][] map = new int[1][l1];
		
		for (int i = 0;i<(path1.length-1);i++)
		{
			double rot1 = direction(path1[i].getPose(),path1[i+1].getPose());
			double[][] nodes1 = place(polygon1,path1[i].getPose().getX(),path1[i].getPose().getY(),rot1);
			for (int j = 0;j<(path2.length-1);j++)
			{
				if ((map[0][i/10]==0))
				{
					double rot2 = direction(path2[j].getPose(),path2[j+1].getPose());
					double[][] nodes2 = place(polygon2,path2[j].getPose().getX(),path2[j].getPose().getY(),rot2);
					if (detectCollision(nodes1,path1[i].getPose(),safe1,dangerous1,nodes2,path2[j].getPose(),safe2,dangerous2))
					{
						map[0][i/10] = 1;
					}
					
				}
			}

		}
		
		double[] cs = new double[2];
		cs[0] = ((double)Allocation.contains(map[0], 1))/l1;
		
		return cs;
	}
	
	public static double[][] place(double[][] polygon,double x,double y,double rotation)
	{
		double[][] nodes = new double[polygon.length][2];
		for (int i = 0;i<polygon.length;i++)
		{
			nodes[i][0] = polygon[i][0]*Math.cos(rotation)-polygon[i][1]*Math.sin(rotation)+x;  
			nodes[i][1] = polygon[i][0]*Math.sin(rotation)+polygon[i][1]*Math.cos(rotation)+y;		
		}
		
		return nodes;
	}

	public static double direction(Pose start,Pose next)
	{
		double x = next.getX() - start.getX();
		double y = next.getY() - start.getY();
		return Math.atan2(y, x);
	}
	
	public static double distancePose(double x1, double y1,double x2, double y2)
	{
		return Math.sqrt(Math.pow((x1-x2),2)+Math.pow((y1-y2),2));
	}
	
	public static boolean detectCollision(double[][] polygon1,Pose center1,double safeDistance1,double dangerousDistance1,double[][] polygon2,Pose center2,double safeDistance2,double dangerousDistance2)
	{
		double dist = distancePose(center1.getX(),center1.getY(),center2.getX(),center2.getY());
		if (dist>(safeDistance1+safeDistance2)) return false;
		if (dist<(dangerousDistance1+dangerousDistance2)) return true;
		double[] param1 = new double[3];
		double[] param2 = new double[3];
		for (int i = 0;i<(polygon1.length-1);i++)
		{
			for (int j = 0;j<(polygon2.length-1);j++)
			{
				param1 = equationLine(polygon1[i],polygon1[i+1]);
				param2 = equationLine(polygon2[j],polygon2[j+1]);
				double[] point = crossLines(param1,param2);
				if (point !=null)if ((pointBetweenPoints(polygon1[i],polygon1[i+1],point))&&(pointBetweenPoints(polygon2[j],polygon1[j+1],point))) return true;
			}
			param1 = equationLine(polygon1[i],polygon1[i+1]);
			param2 = equationLine(polygon2[polygon2.length-1],polygon2[0]);
			double[] point = crossLines(param1,param2);
			if (point !=null)if ((pointBetweenPoints(polygon1[i],polygon1[i+1],point))&&(pointBetweenPoints(polygon2[polygon2.length-1],polygon1[0],point))) return true;
		}
		for (int j = 0;j<(polygon2.length-1);j++)
		{
			param1 = equationLine(polygon1[polygon1.length-1],polygon1[0]);
			param2 = equationLine(polygon2[j],polygon2[j+1]);
			double[] point = crossLines(param1,param2);
			if (point !=null)if ((pointBetweenPoints(polygon1[polygon1.length-1],polygon1[0],point))&&(pointBetweenPoints(polygon2[j],polygon1[j+1],point))) return true;
		}
		param1 = equationLine(polygon1[polygon1.length-1],polygon1[0]);
		param2 = equationLine(polygon2[polygon2.length-1],polygon2[0]);
		double[] point = crossLines(param1,param2);
		if (point !=null)if ((pointBetweenPoints(polygon1[polygon1.length-1],polygon1[0],point))&&(pointBetweenPoints(polygon2[polygon2.length-1],polygon1[0],point))) return true;
		return false;
	}
	
	public static double safeDist(double[][] polygon)
	{
		double max = 0;
		double dist;
		for (int i = 0;i<polygon.length;i++)
		{
			dist = distancePose(0,0,polygon[i][0],polygon[i][1]);
			if (dist>max) max=dist;
		}
		return max;
	}
	
	public static double dangerousDist(double[][] polygon)
	{
		double min = Double.MAX_VALUE;
		double dist;
		double[] pos1 = new double[2];
		double[] pos2 = new double[2];
		final double[] center = {0,0};
		for (int i = 0;i<(polygon.length-1);i++)
		{
			pos1 = polygon[i];
			pos2 = polygon[i+1];
			double[] param = equationLine(pos1,pos2);
			double[] crossPoint = crossLinePoint(param,center);
			if (crossPoint !=null)
			{
				dist = distancePose(0,0,crossPoint[0],crossPoint[1]);
				if (dist<min) min=dist;
			}
		}
		pos1 = polygon[polygon.length-1];
		pos2 = polygon[0];
		double[] param = equationLine(pos1,pos2);
		double[] crossPoint = crossLinePoint(param,center);
		if (crossPoint !=null)
		{
			dist = distancePose(0,0,crossPoint[0],crossPoint[1]);
			if (dist<min) min=dist;
		}
		
		return min;
	}
	
	public static double[] equationLine(double[] pos1, double[] pos2)
	{ 	
		double[] param = new double[3];
		if (pos1[0] == pos2[0])
		{
			param[0] = 1;
			param[1] = 0;
			param[2] = -pos1[0];
		}
		else
		{
			param[0] = (pos2[1]-pos1[1])/(pos2[0]-pos1[0]);
			param[1] = 1;
			param[2] = -(param[0]*pos1[0]+param[1]*pos1[1]);
		}
		return param;
	}
	
	public static double[] perpendicular(double[] param,double[] pos)
	{
		double[] param2 = new double[3];
		if (param[0] == 0)
		{
			param2[0]= 1;
			param2[1]= 0;
		}
		else if (param[1] == 0)
		{
			param2[0]= 0;
			param2[1]= 1;
		}
		else
		{
			param2[0] = -1/(param[0]);
			param2[1]= 1;
		}
		param2[2] = -(param2[0]*pos[0]+param2[1]*pos[1]);
		return param2;
	}
	
	public static double[] crossLines(double[] param,double[] param2)
	{
		double[] cross = new double[2];
		if ((param[0]==param2[0])&&(param[1]==param2[1]))
		{
			return null;
		}
		else if(param[1]==0)
		{
			cross[0]=-param[2];
			cross[1]= - (param2[0]*cross[0]+param2[2]);
			
		}
		else if(param2[1]==0)
		{
			cross[0]=-param2[2];
			cross[1]= - (param[0]*cross[0]+param[2]);
		}
		else
		{
			cross[0] = (param2[2]-param[2])/(param[0]-param2[0]);
			cross[1] = -(param[0]*cross[0]+param[2]);
		}
		return cross;
	}
	public static double[] crossLinePoint(double[] param,double[] point)
	{
		double[] param2 = perpendicular(param,point);
		return crossLines( param,param2);
	}
	
	public static boolean pointBetweenPoints(double[] point1,double[] point2,double[] crossPoint )
	{
		if (point1[0] == point2[0])
		{
			if (isBetween(point1[1],point2[1],crossPoint[1])) return true;
			else return false;
		}
		else
		{
			if (isBetween(point1[0],point2[0],crossPoint[0])) return true;
			else return false;
		}
			
	}
	
	public static boolean isBetween(double x,double y,double p)
	{
		if (x<y)
		{
			if ((p>=x)&&(p<=y)) return true;
			else return false;
					
		}
		else
		{
			if ((p<=x)&&(p>=y)) return true;
			else return false;
		}
	}
	
	
}
