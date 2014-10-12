package irvs.tms.data;

public class Position {
	
	private Float x;
	private Float y;
	private Float z;
	private Float theta;
	
	public Position(Float X, Float Y, Float Z, Float T){
		x = X;
		y = Y;
		z = Z;
		theta = T;
	}
	
	public Float getX(){
		return x;
	}
	
	public Float getY(){
		return y;
	}
	
	public Float getZ(){
		return z;
	}
	
	public Float getTheta(){
		return theta;
	}
	
	@Override
	public String toString(){
		return new String("X:" + x + "Y:" + y + "Z:" + z + "Theta:" + theta);
	}
	
}
