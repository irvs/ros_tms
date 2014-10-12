package irvs.tms.tms_rs_ur_client;
//package irvs.tms.rosandroid_demo_srv_client3;
//
//import android.app.ProgressDialog;
//
//import java.io.File;
//import java.io.FileInputStream;
//import java.io.IOException;
//import java.io.InputStream;
//
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//import android.os.AsyncTask;
//import android.os.Bundle;
//
//import android.widget.AdapterView;
//import android.widget.ArrayAdapter;
//import android.widget.Button;
//import android.widget.EditText;
//import android.widget.LinearLayout;
//import android.widget.Spinner;
//import android.widget.TextView;
//import android.widget.Toast;
//import android.widget.ViewFlipper;
//import android.widget.ImageView;
//import android.util.Log;
//import android.view.MotionEvent;
//import android.view.View.OnClickListener;
//import android.view.View.OnTouchListener;
//import android.view.animation.Animation;
//import android.view.animation.AnimationUtils;
//
//import org.ros.address.InetAddressFactory;
//import org.ros.android.RosActivity;
//import org.ros.node.NodeConfiguration;
//import org.ros.node.NodeMainExecutor;
//
//public class MainActivity extends RosActivity {
//	private ProgressDialog dialog;
//	
//	//place_id決め打ち===================
//	private final int floor_id = 11;
//	private final int bed_id = 12;
//	private final int desk_id = 13;
//	private final int table_id = 14;
//	private final int cabinet1_id = 15;
//	private final int cabinet2_id = 16;
//	//===================================
//	
//	//FTP 決め打ち========================
//	private final String ftp_ip = "192.168.4.171";//4.171//
//	private final String ftp_uid = "tmsftp";//tmsftp//public
//	private final String ftp_pass = "tmsftp";//tmsftp//passftp1990
//	//===================================
//	
//	//object_id 決め打ち==================
//	private final int[] obj_id_arr = {51,52,54,55,56,58,59,60};
//	//===================================
//	
////	private final String savePath = getFilesDir().toString();
//	private final String saveDir = "images";
//	private String savePath;
//	
////	private int objnum = 4;
//	
//	private FtpClient fclient;
//	
//	private int index;
//	private TextView hw;
////	private SmartPal_Srv srvClient;
////	private SrvClient srvClient2;
//	private LinearLayout Window;
//	private TextView tv,ta;
//	private ViewFlipper vf;
//	private Button btnSend,btnRenew,btnADD;
//	private Animation slideInLeft,slideInRight,slideOutLeft,slideOutRight;
//	private Bitmap[] img;
//	private Bitmap noImage;
//	private EditText usr_id;
//	private EditText run_time;
////	private EditText ftp_ip;
//	private Spinner sp;
//
//	public MainActivity() {
//		super("rosandroid_demo_srv_client", "rosandroid_demo_srv_client");
//	}
//
//	@Override
//	public void onCreate(Bundle savedInstanceState) {
//		super.onCreate(savedInstanceState);
//		setContentView(R.layout.main);
//		
//		savePath = "/sdcard/Android/data/irvs.tms.rosandroid_demo_srv_client3/";
//		
//		index = 0;
//		Window = (LinearLayout)findViewById(R.id.Window);
//		hw = (TextView)findViewById(R.id.txtHello);
//		tv = (TextView)findViewById(R.id.txtResult);
//		ta = (TextView)findViewById(R.id.txtANS);
//		vf = (ViewFlipper)findViewById(R.id.flipper);
//		btnSend = (Button)findViewById(R.id.btnSend);
//		btnRenew = (Button)findViewById(R.id.btnRenew);
//		btnADD = (Button)findViewById(R.id.btnADD);
//		slideInLeft = AnimationUtils.loadAnimation(this,R.anim.slide_in_left);
//		slideInRight = AnimationUtils.loadAnimation(this,R.anim.slide_in_right);
//		slideOutLeft = AnimationUtils.loadAnimation(this,R.anim.slide_out_left);
//		slideOutRight = AnimationUtils.loadAnimation(this,R.anim.slide_out_right);
//		usr_id = (EditText)findViewById(R.id.userID);
//		run_time = (EditText)findViewById(R.id.timeID);
////		ftp_ip = (EditText)findViewById(R.id.ftpIP);
//		sp = (Spinner)findViewById(R.id.spinnerPlace);
//		
//		setErrorImage();
//		
//		//place選択用スピナー用placeList作成==========================================================
//		ArrayAdapter<String> plist = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item);
//        plist.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
//        //場所追加:現状決め打ち
//        plist.add("floor");
//        plist.add("bed");
//        plist.add("desk");
//        plist.add("table");
//        plist.add("cabinet1");
//        plist.add("cabinet2");
//        sp.setAdapter(plist);
//        
//        sp.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
//            @Override
//            public void onItemSelected(AdapterView<?> parent, android.view.View view,
//                    int position, long id) {
//                Spinner spinner = (Spinner) parent;
//                // 選択されたアイテムを取得します
//                String item = (String) spinner.getSelectedItem();
//                Toast.makeText(MainActivity.this,item,Toast.LENGTH_LONG).show();
//            }
//            @Override
//            public void onNothingSelected(AdapterView<?> arg0) {
//            }
//        });
//        //==========================================================================================
//	}
//
//	private int getPID(String place){
//		int pid=0;
//		if(place.equals("floor")) pid = floor_id;
//		else if(place.equals("bed")) pid = bed_id;
//		else if(place.equals("desk")) pid = desk_id;
//		else if(place.equals("table")) pid = table_id;
//		else if(place.equals("cabinet1")) pid = cabinet1_id;
//		else if(place.equals("cabinet2")) pid = cabinet2_id;
//		return pid;
//	}
//	
//	@Override
//	protected void init(NodeMainExecutor nodeMainExecutor) {
//		index = 0;
////		srvClient = new SmartPal_Srv();
//////		srvClient2 = new SrvClient();
////		NodeConfiguration nodeConfiguration 
////		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());
////
////		nodeConfiguration.setMasterUri(getMasterUri());
////		nodeMainExecutor.execute(srvClient, nodeConfiguration);  
//		
//		
//		
////		NodeConfiguration nodeConfiguration2 
////		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());
////
////		nodeConfiguration2.setMasterUri(getMasterUri());
////		nodeMainExecutor.execute(srvClient2, nodeConfiguration2);  
//
//		//画面遷移
//		Window.setOnTouchListener(new OnTouchListener(){
//			private float start,end,result;
//			public boolean onTouch(android.view.View v,android.view.MotionEvent event) {
//				result = 0;
//				switch(event.getAction()){
//				case MotionEvent.ACTION_DOWN:
//					start = event.getX();
//					break;
//				case MotionEvent.ACTION_UP:
//					end = event.getX();
//					result = end - start;
//					break;
//				case MotionEvent.ACTION_CANCEL:
//					end = event.getX();
//					result = end - start;
//					break;
//				}
//				if(result>100){
//					vf.setInAnimation(slideInLeft);
//					vf.setOutAnimation(slideOutRight);
//					vf.showPrevious();
//					if(index==0) index = obj_id_arr.length-1;
//					else index--;
//					hw.setText("Object ID." + String.valueOf(obj_id_arr[index]));
//				}
//				else if(result<-100){
//					vf.setInAnimation(slideInRight);
//					vf.setOutAnimation(slideOutLeft);
//					vf.showNext();
//					if(index == obj_id_arr.length-1) index = 0;
//					else index++;
//					hw.setText("Object ID." + String.valueOf(obj_id_arr[index]));
//				}
//				return true;
//			}
//		});
//
//		//service送信
//		btnSend.setOnClickListener(new OnClickListener(){
//			@Override
//			public void onClick(android.view.View v){
//				int uid = Integer.valueOf(usr_id.getText().toString());
//				int did = getPID(sp.getSelectedItem().toString());
//				int time = Integer.valueOf(run_time.getText().toString());
////				int srvResult = srvClient.sendID(obj_id_arr[index],did,uid,time);
////				tv.setText("RESULT:" + String.valueOf(srvResult));
//			}
//		});
//		//FTPで画像ファイル更新
//		btnRenew.setOnClickListener(new OnClickListener(){
//			@Override
//			public void onClick(android.view.View v){
//				dialog = new ProgressDialog(MainActivity.this);
//				dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
//				dialog.setMessage("Download Image Files ...");
//				dialog.show();
//				//無理やり非同期処理
//				AsyncTask<Void,Void,Void> FTPTask = new AsyncTask<Void,Void,Void>(){
//					@Override
//					protected Void doInBackground(Void... params){
//						fclient = new FtpClient(ftp_ip,ftp_uid,ftp_pass);
//						fclient.ChangeDir("2D");//2Dフォルダへ
//						fclient.SetDLPass(savePath,saveDir);//DLフォルダ作成
//						fclient.Refresh();//DLフォルダの中身を削除
//						for(int i=0;i<obj_id_arr.length;i++) fclient.DownLoad(obj_id_arr[i] + ".JPG");//DL
////						fclient.DownloadAll();//imageフォルダの中身を全てダウンロード
//						fclient.close();//解放(ぶっちゃけいらない)
//						return null;
//					}
//					@Override
//					protected void onPostExecute(Void result){
//						dialog.dismiss();
//						//viewFlipperの更新
//						renewViewFlipper();
//					}
//				};
//				//非同期処理実行
//				FTPTask.execute();
//			}
//		});
//		
//		btnADD.setOnClickListener(new OnClickListener(){
//			@Override
//			public void onClick(android.view.View v){
//				long ans = 0;
////				ans = srvClient2.sendSrv(1, 1);
//				ta.setText(String.valueOf(ans));
//			}
//		});
//	}
//	
//	private void renewViewFlipper(){
//		//vchildView削除
//		img = new Bitmap[obj_id_arr.length]; 
//		//viewFlipperセット=========================================================================
//		try {
//			String imname;
//			for(int i=0;i<obj_id_arr.length;i++){
//				imname = String.valueOf(obj_id_arr[i]) + ".JPG";
//				File fimage = new File(savePath + saveDir,imname);
//				InputStream is = new FileInputStream(fimage);
//				img[i] = BitmapFactory.decodeStream(is);
//				is.close();
//				if(img[i] == null) img[i] = noImage;
//				Log.v("MainActivity","PATH:" + fimage.getPath());
//			}
//		}catch(IOException e){
//			hw.setText("READ_ERROR");
//		}
//	
//		for(int i=0;i<obj_id_arr.length;i++){
//			ImageView iv = (ImageView)getLayoutInflater().inflate(R.layout.child, null);
//			iv.setImageBitmap(img[i]);
//			vf.addView(iv);
//		}
//		
//		hw.setText("Object ID."+obj_id_arr[0]);
//		index = 0;
//		//===========================================================================================
//	}
//	
//	private void setErrorImage(){
//		InputStream is;
//		try {
//			is = getResources().getAssets().open("noImage.JPG");
//			noImage = BitmapFactory.decodeStream(is);
//			is.close();
//		} catch (IOException e) {
//			Log.e("MainActivity","FailedSetImage",e);
//		}
//	}
//}
