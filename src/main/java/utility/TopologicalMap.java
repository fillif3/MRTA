package utility;

public class TopologicalMap {
	
	
	private String from = "";
	private String to = "";
	private double rate = 0.0;
	
	public TopologicalMap(String from, String to) {
		this.from = from;
		this.to= to;
	}
	
	public String getFrom() {
		return from;
	}
	
	public String getTo() {
		return to;
	}
	
	
	public void setRate(double rate) {
		this.rate = rate;
	}
	
	public double getRate() {
		return rate;
	}
	
	@Override
	public String toString() {	
		return from + " --> " + to;
	}
	

}
