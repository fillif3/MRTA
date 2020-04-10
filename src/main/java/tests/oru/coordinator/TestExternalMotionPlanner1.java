package tests.oru.coordinator;

import java.io.*;
import java.util.concurrent.TimeUnit;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.logging.Logger;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

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
public class TestExternalMotionPlanner1 {
	
	public static void createAllocations()  throws InterruptedException, IOException{
	
		   Logger logger = Logger.getLogger("MyLog");  
		   FileHandler fh;  

		    try {  

		        // This block configure the logger with handler and formatter  
		        fh = new FileHandler("logs/Filip.log",true);  
		        logger.addHandler(fh);
		        SimpleFormatter formatter = new SimpleFormatter();  
		        fh.setFormatter(formatter);  

		        // the following statement is used to log any messages  
		        

		    } catch (SecurityException e) {  
		        e.printStackTrace();  
		    } catch (IOException e) {  
		        e.printStackTrace();  
		    }  
			
			int set;
			int NUMBER_ROBOTS;
			int simulation;  
			
			
			double MAX_ACCEL = 1.0;
			double MAX_VEL = 4.0;
			double resolution = 0.1;
			//
			//height of the image 
			//Instantiate a trajectory envelope coordinator.
			//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
			// -- the factory method getNewTracker() which returns a trajectory envelope tracker
			// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
			//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
			final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
			
			// FPA: trajectory envelope trackers are no longer visible from the outside, as it
			// FPA: was dangerous to give access to them. They are not really needed anyway for
			// FPA: purposes outside the scope of the coordinator.
			tec.addComparator(new Comparator<RobotAtCriticalSection> () {
				@Override
				public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
					CriticalSection cs = o1.getCriticalSection();
					//RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
					//RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
					RobotReport robotReport1 = o1.getRobotReport();
					RobotReport robotReport2 = o2.getRobotReport();
					return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
				}
			});
			tec.addComparator(new Comparator<RobotAtCriticalSection> () {
				@Override
				public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
					//return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
					return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
				}
			});
					
			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			// FPA: minor API change to make the orders of parameters consistent everywhere...
			//tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
			//tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
			tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

			Coordinate footprint1 = new Coordinate(-1.0,0.5);
			Coordinate footprint2 = new Coordinate(1.0,0.5);
			Coordinate footprint3 = new Coordinate(1.0,-0.5);
			Coordinate footprint4 = new Coordinate(-1.0,-0.5);
			tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
			//Need to setup infrastructure that maintains the representation
			tec.setupSolver(0, 100000000);

			//Setup a simple GUI (null means empty map, otherwise provide yaml file)
			
			//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
			//tec.setVisualization(viz);
			



			tec.setUseInternalCriticalPoints(false);
			tec.setBreakDeadlocks(true);

			//Instantiate a simple motion planner
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();


			for (set=1;set<4;set++)
	        {
	        	for (NUMBER_ROBOTS=3;NUMBER_ROBOTS<10;NUMBER_ROBOTS=NUMBER_ROBOTS+3)
	        	{
	        		for (simulation = 1;simulation<31;simulation++)
	        		{
	        			//if ((set>1)||(NUMBER_ROBOTS>6)||((NUMBER_ROBOTS>3)&&(simulation>10)))
	        			{
	            			logger.info("Map "+set+" with "+NUMBER_ROBOTS+" robots"); 
	            			
	            			String yamlFile = "maps/maptest"+set+".yaml";
	            			String name = "test/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation;
	            			
	            			rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
	            			double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
	            			rsp.setMapResolution(res);
	            			rsp.setTurningRadius(4.0);
	            			rsp.setDistanceBetweenPathPoints(0.05);
	            			rsp.setRadius(0.5);
	            			rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	            			
	            			File file = new File(name); 
	            			  
	            			BufferedReader br = new BufferedReader(new FileReader(file)); 
	            			  
	            			String st;
	            			String[][] RobotsPositions = new String[2][NUMBER_ROBOTS+1];
	            			int index =0;
	            			while ((st = br.readLine()) != null) 
	            			{
	            				RobotsPositions[index] = st.split(" ") ; 
	            				index++;
	            			}
	            			
	            			br.close();
	            			
	            			Pose[] startPoses = new Pose[NUMBER_ROBOTS];
	            			Pose[] goalPoses = new Pose[NUMBER_ROBOTS];

	            			/*Pose startPoseRobot1 = new Pose(32.0,28.0,0.0);
	            			Pose goalPoseRobot1 = new Pose(5.0,1.0,0.0);
	            			Pose startPoseRobot2 = new Pose(2.0,38.0,0.0);
	            			Pose goalPoseRobot2 = new Pose(2.5,3.0,0.0);*/
	            			
	            			String startInfo = "[";
	            			String goalInfo = "[";
	            			for (int i=0;i<NUMBER_ROBOTS;i++)
	            			{
	            				startPoses[i] = new Pose(Double.valueOf(RobotsPositions[0][3*i+1]),Double.valueOf(RobotsPositions[0][3*i+2]),Double.valueOf(RobotsPositions[0][3*i+3]));
	            				goalPoses[i] = new Pose(Double.valueOf(RobotsPositions[1][3*i+1]),Double.valueOf(RobotsPositions[1][3*i+2]),Double.valueOf(RobotsPositions[1][3*i+3]));
	            				startInfo = startInfo + startPoses[i] + " " ;
	            				goalInfo = goalInfo + goalPoses[i] + " " ;
	            			}
	            			startInfo = startInfo + "]";
	            			goalInfo = goalInfo + "]";
	            			logger.info(startInfo);
	            			logger.info(goalInfo);

	            			
	            			
	            			//Place robots in their initial locations (looked up in the data file that was loaded above)
	            			// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
	            			// -- each trajectory envelope has a path of one pose (the pose of the location)
	            			// -- each trajectory envelope is the footprint of the corresponding robot in that pose
	            			
	            			// Creating task allocation
	            			
	            			//int[] alloc = Allocation.allocation(startPoses,goalPoses,rsp,footprint1, footprint2, footprint3, footprint4);
	            			
	            			Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
	            			
	            			 double[] elapsedTime = new double[3];
	            			 
	            			 int[] differences = new int[3];
	            			 
	            			 int[][] alloc = new int[3][NUMBER_ROBOTS];
	            			
	            			 double startTime = System.currentTimeMillis();
	            			
	            			 alloc[0] = Allocation.Eucalidian(startPoses, goalPoses);
	            			 
	            			 double finishTime = System.currentTimeMillis();
	            			 
	            			 elapsedTime[0] = finishTime - startTime;
	            			 
	            			 String helper = "According to Eucalidian distance, the robots to tasks are allocated as follows:";
	            			 
	            			 for (int i=0;i<NUMBER_ROBOTS;i++)
	            			 {
	            				helper+= i+"->"+alloc[0][i]+",";

	            			 }
	            			 
	            			 logger.info(helper);
	            			 
	            			 startTime = System.currentTimeMillis();
	            			 
	            			 List<List<PoseSteering[]>> paths = Allocation.checkPaths(startPoses,goalPoses,rsp,fpGeom);
	            			 
	            			 double[][] distances = Allocation.distance(paths);
	            			 
	            			 alloc[1] = Allocation.predict(paths,distances);
	            			 
	            			 
	            			 
	            			 finishTime = System.currentTimeMillis();
	            			 
	            			 elapsedTime[1] = finishTime - startTime;
	            			 
	            			 helper = "According to distance of path, the robots to tasks are allocated as follows:";
	            			 
	            			 for (int i=0;i<NUMBER_ROBOTS;i++)
	            			 {
	            				helper+= i+"->"+alloc[1][i]+",";

	            			 }
	            			 logger.info(helper);
	            			 
	            			 differences[0] = Allocation.checkDifferences(alloc[0],alloc[1]);
	            			 
	            			 startTime = System.currentTimeMillis();
	            			 
	            			 alloc[2]= Allocation.correct(paths,distances,alloc[1], footprint1, footprint2, footprint3, footprint4);
	            			
	            			 finishTime = System.currentTimeMillis();
	            			 
	            			 elapsedTime[2] = finishTime - startTime +elapsedTime[1];
	            			 
	            			 helper = "According to correction, the robots to tasks are allocated as follows:";
	            			 
	            			 for (int i=0;i<NUMBER_ROBOTS;i++)
	            			 {
	            				helper+= i+"->"+alloc[2][i]+",";

	            			 }
	            			 logger.info(helper);
	            			 
	            			 differences[1] = Allocation.checkDifferences(alloc[1],alloc[2]);
	            			 
	            			 name = "allocations/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation;
	        				 
	        				 try {
	    	        	            // Assume default encoding.
	    	        	            FileWriter fileWriter =
	    	        	                new FileWriter(name);

	    	        	            // Always wrap FileWriter in BufferedWriter.
	    	        	            BufferedWriter bufferedWriter =
	    	        	                new BufferedWriter(fileWriter);

	    	        	            // Note that write() does not automatically
	    	        	            // append a newline character.
	    	        	            
	    	        	            for (int i=0;i<3;i++)
	    	        	            {
	    	        	            	bufferedWriter.write("method"+i);
	    	 	        	            bufferedWriter.newLine();
	    	 	        	            bufferedWriter.write("elapsed time:"+elapsedTime[i]);
	    	 	        	            bufferedWriter.newLine();
	    	 	        	            bufferedWriter.write("allocation:");
	    	 	        	            for (int j=0;j<NUMBER_ROBOTS;j++)
	    	 	        	            {
	    	 	        	            	bufferedWriter.write(" "+alloc[i][j]);
	    	 	        	            	
	    	 	        	            }
	    	 	        	            	
	    	 	        	            bufferedWriter.newLine();
	    	        	            }
	    	        	            

	    	        	            // Always close files.
	    	        	            bufferedWriter.close();
	    	        	        }
	    	        	        catch(IOException ex) {
	    	        	            System.out.println(
	    	        	                "Error writing to file '"
	    	        	                + name + "'");
	    	        	            // Or we could just do this:
	    	        	            // ex.printStackTrace();
	    	        	        }
	        			}
	        			

	        		}
	        	}
	        		
	        }
			
			
	  
	        // For storing image in RAM 
			


			/*for (int i=0;i<NUMBER_ROBOTS;i++)
			{
				for (int j=0;j<NUMBER_ROBOTS;j++)
				{
					if (paths.get(i).get(j)==null)
					{
						logger.info("No path between " + (i+1) + " robot and " +(j+1)+" destination");
					}

				}

			}
			 
			int[] robotIds = new int[NUMBER_ROBOTS];
	        for (int r = 1; r <= NUMBER_ROBOTS; r++) {
	            robotIds[r - 1] = r;
	        }
			
			boolean flag = true; 
			
			try
			{
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					
					Missions.enqueueMission(new Mission(i+1, paths.get(i).get(alloc[i])));
					///Missions.
					
					
		
				}
			}
			catch(Exception e)
			{
				flag = false;
			}
			boolean[] freeRobots = new boolean[NUMBER_ROBOTS];
			
			startTime = System.currentTimeMillis();
			
			if (flag)
			{
				
				for (int j=0;j<1;j++) {
					
					
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						Missions.dequeueMission(i+1);
						Missions.enqueueMission(new Mission(i+1, paths.get(i).get(alloc[i])));
			
					}
				
				
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						tec.placeRobot(i+1, startPoses[i]);
		
					}
					
					
					
					Missions.startMissionDispatchers(tec, false, robotIds);
			
					
					
					
						while(true)
						{
							for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
								
							
							if (Allocation.contains(freeRobots, false)>0)
							{
								
								System.out.println("end");
								break;
							}
						}
						
						while(true)
						{
							for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
								
							if (Allocation.contains(freeRobots, false)==0)
							{
								
								System.out.println("koniec ");
								break;
							}
						}
				
				}
				finishTime = System.currentTimeMillis();
				 
				 elapsedTime[4] = (finishTime - startTime)/5;
			}
			else
			{
				elapsedTime[4] = -1;
				logger.info("Eucalidian method found no path");
			}
			
			if (differences[0]>0)
			{
				startTime = System.currentTimeMillis();
				for(int j=0;j<1;j++) {
					
				
				
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						tec.placeRobot(i+1, startPoses[i]);
				
					}
				
					try {
						for (int i=0;i<NUMBER_ROBOTS;i++)
						{
							Missions.dequeueMission(i+1);
							Missions.enqueueMission(new Mission(i+1, paths.get(i).get(prediction[i])));
				
						}
					}
					catch(Exception e)
					{
						flag = false;
					}
						System.out.println("Added missions " + Missions.getMissions());
					
					
					//Start a mission dispatching thread for each robot, which will run forever
						Missions.startMissionDispatchers(tec, false, robotIds);
				
					
					
					 
					 
					 
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						
						if (Allocation.contains(freeRobots, false)>0)
						{
							
							System.out.println("end");
							break;
						}
					}
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						if (Allocation.contains(freeRobots, false)==0)
						{
							
							System.out.println("koniec ");
							break;
						}
					}
				}
				
				finishTime = System.currentTimeMillis();
				 
				 elapsedTime[5] = (finishTime - startTime)/5;
			 }
			
			if (differences[1]>0)
			{
			 startTime = System.currentTimeMillis();
			 for(int j=0;j<1;j++) {
					
				
				
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						tec.placeRobot(i+1, startPoses[i]);
				
					}
				
					try {
						for (int i=0;i<NUMBER_ROBOTS;i++)
						{
							Missions.dequeueMission(i+1);
							Missions.enqueueMission(new Mission(i+1, paths.get(i).get(correction[i])));
				
						}
					}
					catch(Exception e)
					{
						flag = false;
					}
						System.out.println("Added missions " + Missions.getMissions());
					
					
					//Start a mission dispatching thread for each robot, which will run forever
						Missions.startMissionDispatchers(tec, false, robotIds);
				
					
					
					 
					 
					 
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						
						if (Allocation.contains(freeRobots, false)>0)
						{
							
							System.out.println("end");
							break;
						}
					}
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						if (Allocation.contains(freeRobots, false)==0)
						{
							
							System.out.println("koniec ");
							break;
						}
					}
				}
				
				finishTime = System.currentTimeMillis();
				 
				 elapsedTime[6] = (finishTime - startTime)/5;
			 
			 logger.info("Output time is: "+(elapsedTime[0]/1000) + ", "+(elapsedTime[1]/1000) + ", "+(elapsedTime[2]/1000) + ", "+(elapsedTime[3]/1000) + ", "+(elapsedTime[4]/1000) + ", "+(elapsedTime[5]/1000) + ", ");
			}
			if (differences[2]>0)
			{
			 startTime = System.currentTimeMillis();
			 for(int j=0;j<1;j++) {
					
				
				
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						tec.placeRobot(i+1, startPoses[i]);
				
					}
				
					try {
						for (int i=0;i<NUMBER_ROBOTS;i++)
						{
							Missions.dequeueMission(i+1);
							Missions.enqueueMission(new Mission(i+1, paths.get(i).get(correction2[i])));
				
						}
					}
					catch(Exception e)
					{
						flag = false;
					}
						System.out.println("Added missions " + Missions.getMissions());
					
					
					//Start a mission dispatching thread for each robot, which will run forever
						Missions.startMissionDispatchers(tec, false, robotIds);
				
					
					
					 
					 
					 
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						
						if (Allocation.contains(freeRobots, false)>0)
						{
							
							System.out.println("end");
							break;
						}
					}
					
					while(true)
					{
						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
							
						if (Allocation.contains(freeRobots, false)==0)
						{
							
							System.out.println("koniec ");
							break;
						}
					}
				}
				
				finishTime = System.currentTimeMillis();
				 
				 elapsedTime[7] = (finishTime - startTime)/5;
			 
			 logger.info("Output time is: "+(elapsedTime[0]/1000) + ", "+(elapsedTime[1]/1000) + ", "+(elapsedTime[2]/1000) + ", "+(elapsedTime[3]/1000) + ", "+(elapsedTime[4]/1000) + ", "+(elapsedTime[5]/1000)+", "+(elapsedTime[6]/1000)+", "+(elapsedTime[7]/1000));
			}*/

		
	}

	public static void runCoord() throws InterruptedException, IOException {
		
	   Logger logger = Logger.getLogger("MyLog");  
	   FileHandler fh;  

	    try {  

	        // This block configure the logger with handler and formatter  
	        fh = new FileHandler("logs/Filip.log",true);  
	        logger.addHandler(fh);
	        SimpleFormatter formatter = new SimpleFormatter();  
	        fh.setFormatter(formatter);  

	        // the following statement is used to log any messages  
	        

	    } catch (SecurityException e) {  
	        e.printStackTrace();  
	    } catch (IOException e) {  
	        e.printStackTrace();  
	    }  
		
		int set;
		int NUMBER_ROBOTS;
		int simulation;  
		int method;
		
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		double resolution = 0.1;
		//
		//height of the image 
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
		// FPA: trajectory envelope trackers are no longer visible from the outside, as it
		// FPA: was dangerous to give access to them. They are not really needed anyway for
		// FPA: purposes outside the scope of the coordinator.
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				//RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				//RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				//return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
				
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		// FPA: minor API change to make the orders of parameters consistent everywhere...
		//tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		//tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		


		tec.setUseInternalCriticalPoints(false);
		tec.setBreakDeadlocks(true);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		
		String name = "simulations/helper";
		
		File file = new File(name);

		BufferedReader br = new BufferedReader(new FileReader(file)); 
		  
		String[] helpString = new String[5];
		int[] helper = new int[5];
		String st;
		while ((st = br.readLine()) != null) 
		{
			helpString = st.split(" "); 
		}
		for (int i=0;i<5;i++)
		{
			helper[i] = Integer.valueOf(helpString[i]);
		}
		
		
		br.close();
		
		set = helper[4]+1;
		NUMBER_ROBOTS = (helper[3]+1)*3;
		simulation = helper[2]+1;
		method = helper[1];
		
		
		
		//kkkkkkkkkkkkkkkkkkkk
		
		//for (set=1;set<4;set++)
        {
        	//for (NUMBER_ROBOTS=3;NUMBER_ROBOTS<10;NUMBER_ROBOTS=NUMBER_ROBOTS+3)
        	{
        		//for (simulation = 1;simulation<31;simulation++)
        		{
        			logger.info("Map "+set+" with "+NUMBER_ROBOTS+" robots"); 
        			
        			String yamlFile = "maps/maptest"+set+".yaml";
        			
        			 name = "test/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation;
    				
        			 file = new File(name); 
        			  
        			br = new BufferedReader(new FileReader(file)); 
        			  

        			String[][] RobotsPositions = new String[2][NUMBER_ROBOTS+1];
        			int index =0;
        			while ((st = br.readLine()) != null) 
        			{
        				RobotsPositions[index] = st.split(" ") ; 
        				index++;
        			}
        			
        			br.close();
        			
        			name = "allocations/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation;
     				
         			file = new File(name); 
         			  
         			br = new BufferedReader(new FileReader(file)); 
         			  
         			String[][] allocString = new String[3][NUMBER_ROBOTS+1];
         			index =0;
         			while ((st = br.readLine()) != null) 
         			{
         				index++;
         				if(index%3==0)
         				{
         					allocString[index/3-1] = st.split(" ") ; 
         				}
         				
         			}
         			
         			br.close();
         			
         			Pose[] startPoses = new Pose[NUMBER_ROBOTS];
        			Pose[] goalPoses = new Pose[NUMBER_ROBOTS];

        			/*Pose startPoseRobot1 = new Pose(32.0,28.0,0.0);
        			Pose goalPoseRobot1 = new Pose(5.0,1.0,0.0);
        			Pose startPoseRobot2 = new Pose(2.0,38.0,0.0);
        			Pose goalPoseRobot2 = new Pose(2.5,3.0,0.0);*/
        			
        			String startInfo = "[";
        			String goalInfo = "[";
        			for (int i=0;i<NUMBER_ROBOTS;i++)
        			{
        				startPoses[i] = new Pose(Double.valueOf(RobotsPositions[0][3*i+1]),Double.valueOf(RobotsPositions[0][3*i+2]),Double.valueOf(RobotsPositions[0][3*i+3]));
        				goalPoses[i] = new Pose(Double.valueOf(RobotsPositions[1][3*i+1]),Double.valueOf(RobotsPositions[1][3*i+2]),Double.valueOf(RobotsPositions[1][3*i+3]));
        				startInfo = startInfo + startPoses[i] + " " ;
        				goalInfo = goalInfo + goalPoses[i] + " " ;
        			}
        			startInfo = startInfo + "]";
        			goalInfo = goalInfo + "]";
        			logger.info(startInfo);
        			logger.info(goalInfo);
        			
        			int[][] alloc = new int[3][NUMBER_ROBOTS];
    				
    				for (int i=0;i<NUMBER_ROBOTS;i++)
        			{
    					for (int j=0;j<3;j++)
    					{
    						alloc[j][i] = Integer.valueOf(allocString[j][i+1]);
    						//startInfo = startInfo + startPoses[i] + " " ;
    					}
        			}
        			
        			//if ((set>1)||(NUMBER_ROBOTS>6)||((NUMBER_ROBOTS>3)&&(simulation>10)))
    				
    				double elapsedTime = 0;
        			//for (method = 0;method<3;method++)
        			{
        				
        				boolean flag =true;
        				
        				for (int i=0;i<method;i++)
        				{
        					int difference = Allocation.checkDifferences(alloc[i],alloc[method]);
        					if (difference == 0)
        					{
        						flag = false;
        						
        						name = "simulations/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation+"method"+i+"approach"+helper[0];
        						
        						file = new File(name); 
        	         			  
        	         			br = new BufferedReader(new FileReader(file)); 
        	         			  
        	         			double time=0;
        	         			while ((st = br.readLine()) != null) 
        	         			{
        	
        	         				time = Double.valueOf(st) ; 
        	         				
        	         				
        	         			}
        	         			
        	         			br.close();
        						
        						elapsedTime = time;
        						
        					}
        		
        				}
            			
        				//
        				//{
        				//	return;
        				//}
        				if(flag)
        				{
	            			rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
	            			double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
	            			rsp.setMapResolution(res);
	            			rsp.setTurningRadius(4.0);
	            			rsp.setDistanceBetweenPathPoints(0.05);
	            			rsp.setRadius(0.5);
	            			rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
	            			
	            			JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
	        				tec.setVisualization(viz);
	            			
	            			
	            			 
	            			 
	            			 int[] robotIds = new int[NUMBER_ROBOTS];
	        		        for (int r = 1; r <= NUMBER_ROBOTS; r++) {
	        		            robotIds[r - 1] = r;
	        		        }
	        		        
	            			Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
	        		        
	        		        for (int i=0;i<NUMBER_ROBOTS;i++)
	        				{
	        		        	rsp.clearObstacles();
	        		        	rsp.setStart(startPoses[i]);
	        					rsp.setGoals(goalPoses[alloc[method][i]]);
	        					for (int k=0;k<NUMBER_ROBOTS;k++)
	        					{
	        						if (k!=i) 
	        							{
	        							rsp.addObstacles(fpGeom, startPoses[k]);
	        							rsp.addObstacles(fpGeom, goalPoses[alloc[method][k]]);
	        							}
	        					}
	        					rsp.plan();
	        					PoseSteering[] path = rsp.getPath();
	
	        					Missions.enqueueMission(new Mission(i+1, path));
	        					///Missions.
	        		
	        				}
	        		        
	        		        for (int i=0;i<NUMBER_ROBOTS;i++)
	        				{
	        					tec.placeRobot(i+1, startPoses[i]);
	        	
	        				}
	        				
	        				
	        				Missions.startMissionDispatchers(tec, false, robotIds);
	        				
	        				boolean[] freeRobots = new boolean[NUMBER_ROBOTS];
	
	        				double startTime = System.currentTimeMillis();
	        		        
	        				while(true)
	    					{
	    						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
	    							
	    						
	    						if (Allocation.contains(freeRobots, false)>0)
	    						{
	    							
	    							System.out.println("end");
	    							break;
	    						}
	    					}
	    					
	    					while(true)
	    					{
	    						for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
	    							
	    						if (Allocation.contains(freeRobots, false)==0)
	    						{
	    							
	    							System.out.println("koniec ");
	    							break;
	    						}
	    						
	    						
	    					}
	    					
	    					double finishTime = System.currentTimeMillis();
	    					 
	    					 elapsedTime = (finishTime - startTime);///5;
        				}

        			}
        			name = "simulations/map"+set+"rob"+NUMBER_ROBOTS+"set"+simulation+"method"+method+"approach"+helper[0];
        			try {
        	            // Assume default encoding.
        	            FileWriter fileWriter =
        	                new FileWriter(name);

        	            // Always wrap FileWriter in BufferedWriter.
        	            BufferedWriter bufferedWriter =
        	                new BufferedWriter(fileWriter);

        	            // Note that write() does not automatically
        	            // append a newline character.


 	        	            bufferedWriter.write(elapsedTime+"");
        	            
        	            

        	            // Always close files.
        	            bufferedWriter.close();
        	        }
        	        catch(IOException ex) {
        	            System.out.println(
        	                "Error writing to file '"
        	                + name + "'");
        	            // Or we could just do this:
        	            // ex.printStackTrace();
        	        }
        			
        			name = "simulations/helper";
        			        			
        			
        			helper[0]++;
        			if (helper[0]==3)
        			{
        				helper[0]=0;
        				helper[1]++;
        				if(helper[1]==3)
        				{
        					helper[1]=0;
        					helper[2]++;
        					if(helper[2]==30)
            				{
            					helper[2]=0;
            					helper[3]++;
            					if(helper[3]==3)
                				{
                					helper[3]=0;
                					helper[4]++;
                					
                				}
            				}
        				}
        			}
        			
        			
        			
        			
        			
        			try {
        	            // Assume default encoding.
        	            FileWriter fileWriter =
        	                new FileWriter(name);

        	            // Always wrap FileWriter in BufferedWriter.
        	            BufferedWriter bufferedWriter =
        	                new BufferedWriter(fileWriter);

        	            // Note that write() does not automatically
        	            // append a newline character.
        	            
        	            
        	            	bufferedWriter.write(helper[0]+" "+helper[1]+" "+helper[2]+" "+helper[3]+" "+helper[4]);
 	        	           
        	            
        	            

        	            // Always close files.
        	            bufferedWriter.close();
        	        }
        	        catch(IOException ex) {
        	            System.out.println(
        	                "Error writing to file '"
        	                + name + "'");
        	            // Or we could just do this:
        	            // ex.printStackTrace();
        	        }
        			
        			System.exit(1);

        		}
        	}
        		
        }
		
		
  
        // For storing image in RAM 
		


		/*for (int i=0;i<NUMBER_ROBOTS;i++)
		{
			for (int j=0;j<NUMBER_ROBOTS;j++)
			{
				if (paths.get(i).get(j)==null)
				{
					logger.info("No path between " + (i+1) + " robot and " +(j+1)+" destination");
				}

			}

		}
		 
		int[] robotIds = new int[NUMBER_ROBOTS];
        for (int r = 1; r <= NUMBER_ROBOTS; r++) {
            robotIds[r - 1] = r;
        }
		
		boolean flag = true; 
		
		try
		{
			for (int i=0;i<NUMBER_ROBOTS;i++)
			{
				
				Missions.enqueueMission(new Mission(i+1, paths.get(i).get(alloc[i])));
				///Missions.
				
				
	
			}
		}
		catch(Exception e)
		{
			flag = false;
		}
		boolean[] freeRobots = new boolean[NUMBER_ROBOTS];
		
		startTime = System.currentTimeMillis();
		
		if (flag)
		{
			
			for (int j=0;j<1;j++) {
				
				
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					Missions.dequeueMission(i+1);
					Missions.enqueueMission(new Mission(i+1, paths.get(i).get(alloc[i])));
		
				}
			
			
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					tec.placeRobot(i+1, startPoses[i]);
	
				}
				
				
				
				Missions.startMissionDispatchers(tec, false, robotIds);
		
				
				
				
					
			
			}
			finishTime = System.currentTimeMillis();
			 
			 elapsedTime[4] = (finishTime - startTime)/5;
		}
		else
		{
			elapsedTime[4] = -1;
			logger.info("Eucalidian method found no path");
		}
		
		if (differences[0]>0)
		{
			startTime = System.currentTimeMillis();
			for(int j=0;j<1;j++) {
				
			
			
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					tec.placeRobot(i+1, startPoses[i]);
			
				}
			
				try {
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						Missions.dequeueMission(i+1);
						Missions.enqueueMission(new Mission(i+1, paths.get(i).get(prediction[i])));
			
					}
				}
				catch(Exception e)
				{
					flag = false;
				}
					System.out.println("Added missions " + Missions.getMissions());
				
				
				//Start a mission dispatching thread for each robot, which will run forever
					Missions.startMissionDispatchers(tec, false, robotIds);
			
				
				
				 
				 
				 
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					
					if (Allocation.contains(freeRobots, false)>0)
					{
						
						System.out.println("end");
						break;
					}
				}
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					if (Allocation.contains(freeRobots, false)==0)
					{
						
						System.out.println("koniec ");
						break;
					}
				}
			}
			
			finishTime = System.currentTimeMillis();
			 
			 elapsedTime[5] = (finishTime - startTime)/5;
		 }
		
		if (differences[1]>0)
		{
		 startTime = System.currentTimeMillis();
		 for(int j=0;j<1;j++) {
				
			
			
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					tec.placeRobot(i+1, startPoses[i]);
			
				}
			
				try {
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						Missions.dequeueMission(i+1);
						Missions.enqueueMission(new Mission(i+1, paths.get(i).get(correction[i])));
			
					}
				}
				catch(Exception e)
				{
					flag = false;
				}
					System.out.println("Added missions " + Missions.getMissions());
				
				
				//Start a mission dispatching thread for each robot, which will run forever
					Missions.startMissionDispatchers(tec, false, robotIds);
			
				
				
				 
				 
				 
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					
					if (Allocation.contains(freeRobots, false)>0)
					{
						
						System.out.println("end");
						break;
					}
				}
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					if (Allocation.contains(freeRobots, false)==0)
					{
						
						System.out.println("koniec ");
						break;
					}
				}
			}
			
			finishTime = System.currentTimeMillis();
			 
			 elapsedTime[6] = (finishTime - startTime)/5;
		 
		 logger.info("Output time is: "+(elapsedTime[0]/1000) + ", "+(elapsedTime[1]/1000) + ", "+(elapsedTime[2]/1000) + ", "+(elapsedTime[3]/1000) + ", "+(elapsedTime[4]/1000) + ", "+(elapsedTime[5]/1000) + ", ");
		}
		if (differences[2]>0)
		{
		 startTime = System.currentTimeMillis();
		 for(int j=0;j<1;j++) {
				
			
			
				for (int i=0;i<NUMBER_ROBOTS;i++)
				{
					tec.placeRobot(i+1, startPoses[i]);
			
				}
			
				try {
					for (int i=0;i<NUMBER_ROBOTS;i++)
					{
						Missions.dequeueMission(i+1);
						Missions.enqueueMission(new Mission(i+1, paths.get(i).get(correction2[i])));
			
					}
				}
				catch(Exception e)
				{
					flag = false;
				}
					System.out.println("Added missions " + Missions.getMissions());
				
				
				//Start a mission dispatching thread for each robot, which will run forever
					Missions.startMissionDispatchers(tec, false, robotIds);
			
				
				
				 
				 
				 
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					
					if (Allocation.contains(freeRobots, false)>0)
					{
						
						System.out.println("end");
						break;
					}
				}
				
				while(true)
				{
					for(int i=0;i<NUMBER_ROBOTS;i++) freeRobots[i] = tec.isFree(i+1);
						
					if (Allocation.contains(freeRobots, false)==0)
					{
						
						System.out.println("koniec ");
						break;
					}
				}
			}
			
			finishTime = System.currentTimeMillis();
			 
			 elapsedTime[7] = (finishTime - startTime)/5;
		 
		 logger.info("Output time is: "+(elapsedTime[0]/1000) + ", "+(elapsedTime[1]/1000) + ", "+(elapsedTime[2]/1000) + ", "+(elapsedTime[3]/1000) + ", "+(elapsedTime[4]/1000) + ", "+(elapsedTime[5]/1000)+", "+(elapsedTime[6]/1000)+", "+(elapsedTime[7]/1000));
		}*/

	}

	public static void main(String[] args) throws InterruptedException, IOException {

		runCoord();
		//createAllocations();
	}
}

