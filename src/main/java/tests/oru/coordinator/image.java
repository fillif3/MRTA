package tests.oru.coordinator;

import java.io.File; 
import java.io.IOException; 
import java.awt.image.BufferedImage; 
import javax.imageio.ImageIO; 
import java.awt.Color;
import java.awt.Graphics;
import java.io.File;
import java.util.Comparator;
import java.util.Random;

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

public class image {
	
	// wihte = -1
	// black = 0
	
	public static BufferedImage copyImage(BufferedImage source){
	    BufferedImage b = new BufferedImage(source.getWidth(), source.getHeight(), source.getType());
	    Graphics g = b.getGraphics();
	    g.drawImage(source, 0, 0, null);
	    g.dispose();
	    return b;
	}
	
	public static BufferedImage dilatation(int width,int height,BufferedImage image,double distance, double resolution ) 
	{
		int distPixels = (int) Math.ceil(distance/resolution);
		
		BufferedImage dimg = copyImage( image); 
		
        for(int i=0;i<width;i++)
        {
            for(int j=0;j<height;j++)
            {
            	if(checkDilatation(width,height,image,distPixels,i,j))
            	{
            		dimg.setRGB(i, j, Color.BLACK.getRGB());//);
            	}
            	
            }
        }
        
        return dimg;
		
	}
	
	public static Pose[] createFreePoses(int width, int height, BufferedImage image,int number,double resolution)
	{
		int index = 0;
		BufferedImage cimage = copyImage(image);
		Pose[] out = new Pose[number];
		Random r = new Random();
		while(index<number)
		{
			int rand_width = r.nextInt(width); 
			int rand_height = r.nextInt(height); 
			if (cimage.getRGB(rand_width, rand_height)==-1)
			{
				int x = rand_width;
				int y = height - rand_height;
				double xDist = ((double)x)*resolution;
				double yDist = ((double)y)*resolution;
				double angle = r.nextDouble()*2*Math.PI; 
				out[index] = new Pose(xDist,yDist,angle);
				index++;
				cimage=blackCircle(cimage,Math.sqrt(5),resolution,width,height,rand_width,rand_height);
			}
		}
		 
        
		return out;
	}

    private static BufferedImage blackCircle(BufferedImage image, double dist, double resolution,int width,int height,int x,int y) {
    	int distPixels = (int) Math.ceil(dist/resolution);
    	int x1,x2,y1,y2;
    	if (x<distPixels) x1=0;
    	else x1 = x-distPixels;
    	if (y<distPixels) y1=0;
    	else y1 = y-distPixels;
		if ((x+distPixels)>=width) x2 = width;
		else x2 = x+distPixels;
		if ((y+distPixels)>=height) y2=height;
		else y2 = y+distPixels;
		for(int i=x1;i<x2;i++)
        {
        	for(int j=y1;j<y2;j++)
            {
            	if(Allocation.distancePose(x, y, i, j)<distPixels)
            	{
            		image.setRGB(i, j,Color.BLACK.getRGB());
            	}
            	
            }
        }
		
		return image;
		
	}

	private static boolean checkDilatation(int width, int height, BufferedImage image, int distPixels,int x,int y) {
		if (image.getRGB(x, y)==0) return false;
		if (x<distPixels) return true;
		if (y<distPixels) return true;
		if ((x+distPixels)>=width) return true;
		if ((y+distPixels)>=height) return true;
        for(int i=(x-distPixels);i<=(x+distPixels);i++)
        {
        	for(int j=(y-distPixels);j<=(y+distPixels);j++)
            {
            	if((image.getRGB(i, j)==Color.BLACK.getRGB()))
            	{
            		return true;
            	}
            	
            }
        }
        return false;

	}

	public static void main(String args[])throws IOException 
    { 
        int width = 800;    //width of the image 
        int height = 440;   //height of the image 
  
        // For storing image in RAM 
        BufferedImage image = null; 
        BufferedImage image2 = null; 
  
        // READ IMAGE 
        try
        { 
            File input_file = new File("maps/maptest3.png"); //image file path 
  
            /* create an object of BufferedImage type and pass 
               as parameter the width,  height and image int 
               type.TYPE_INT_ARGB means that we are representing 
               the Alpha, Red, Green and Blue component of the 
               image pixel using 8 bit integer value. */
            image = new BufferedImage(width, height, 
                                    BufferedImage.TYPE_BYTE_BINARY);
            
  
             // Reading input file 
            image = ImageIO.read(input_file); 
  
            System.out.println("Reading complete."); 
        } 
        catch(IOException e) 
        { 
            System.out.println("Error: "+e); 
        } 
       image = dilatation(width,height,image,2,0.1);
       


        
        // WRITE IMAGE 
        try
        { 
            // Output file path 
            File output_file = new File("maps/maptest31dil1.png"); 
  
            // Writing to file taking type and path as 
            ImageIO.write(image, "png", output_file); 
  
            System.out.println("Writing complete."); 
        } 
        catch(IOException e) 
        { 
            System.out.println("Error: "+e); 
        } 
        

    }//main() ends here 
}//class ends here 

