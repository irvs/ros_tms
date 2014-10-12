package irvs.tms.data;

import android.util.Log;

/*
 * オブジェクト
 * あるオブジェクト一つを表す
 * 自分の上位，下位のオブジェクトのパスを持つ
 * これは描画の際にオブジェクトを引数として描画したいから
 */
public class Object {
	//オブジェクト名
	private String objName;
	//オブジェクトID
	private int objId;
	//オブジェクトの種類
	private int objType;
	//オブジェクトの画像のパス・・・アイコンサイズのBmpを持ってもいいかも
	private String objImgPath;
	//オブジェクトの上位のオブジェクト
	private int srcId;
	private String srcName;
	//オブジェクトの位置
	private Position p;
	//オブジェクトの状態
	private int objState;
	

	public Object(String name, int id){
		objName = new String(name);
		objId = id;
		objImgPath = new String();
		srcName = new String();
//		dstObject = new ArrayList<Object>(); 
	}

	/*public Object(String name, int id,int type, String imgPath){
		objName = new String(name);
		objId = id;
		objType = type;
		objImgPath = new String(imgPath);
		dstObject = new ArrayList<Object>(); 
	}*/
	
	public Object(String name, int id, int type, int state){
		objName = new String(name);
		objId = id;
		objType = type;
		objState = state;
		objImgPath = new String();
		srcName = new String();
//		dstObject = new ArrayList<Object>(); 
	}

	@Override
	public String toString(){
		if(this != null){
			return new String("Name:" + objName + "\nId:" + objId + "\nImg:" + objImgPath + "\nSrc:" + srcName + "\n");
		}
		else {
			Log.v("Object.java","Nothing Object");
			return new String("Nothing Object");
		}
	}

	public String getName(){
		return objName;
	}

	public int getId(){
		return objId;
	}
	
	public int getType(){
		return objType;
	}

	
	public int getState(){
		return objState;
	}
	
	public int getSrcId(){
		return srcId;
	}
	
	public String getSrcName(){
		return srcName;
	}

	public void setName(String name){
		objName = name;
	}

	public void setId(int id){
		objId = id;
	}

	public void setType(int type){
		objType = type;
	}
	
	public void setSrc(int id, String name){
		srcId = id;
		srcName = name;
	}
	

	public void setPosition(Float X,Float Y,Float Z,Float T){
		p = null;
		p = new Position(X,Y,Z,T);
	}

	public Position getPosition(){
		return p;
	}
	
	public void reset(String name, int id){
		objName = new String(name);
		objId = id;
	}

}
