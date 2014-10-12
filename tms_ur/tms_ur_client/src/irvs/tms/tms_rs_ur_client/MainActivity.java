//RS_CLIENT1.10(数字に意味はない)
//無理やりリアルタイムで更新
//Object currentObject;に入ったObjectエクスプローラ定期更新版
//時間が立つと何故かうまく動かなくなる
//ハンドラの生成・破棄がうまくできていない？
//初めにSENDボタンを押したら定期更新がスタート
//デモの時はFTP（RENEWボタン、SETボタン）はいじらないほうが無難
//(設定が合ってなくて画像が消える恐れあり。というか現状合ってない）
//ボタン押下後3秒毎に定期更新をかける
//ボタン自体は即時に反応するはずだがDBとの通信でタイムラグあり
//定期更新も上に同じ
//ポップアップ時、アプリ非アクティブ時は定期更新ハンドラを破棄
//アプリ中断後定期更新を再会するには再びSENDする必要がある（仕様）

package irvs.tms.tms_rs_ur_client;


import irvs.tms.ftp.FtpClient;
import irvs.tms.srv.GetAroundObject;
import irvs.tms.srv.GetObjectInfoRt;
import irvs.tms.srv.RenewSrv;
import irvs.tms.data.Object;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;

import android.os.Message;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TextView;
import android.widget.TextView.BufferType;
import android.widget.Toast;
import android.widget.ViewFlipper;



import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

	//Setting関連==================================
	private static int USERID;
	private static String FTPIP,FTPUSR,FTPPASS;
	private Button btnSetting;

	private final int UID = 1;
	private final String FIP = "192.168.4.170";
	private final String FU = "tmsftp";
	private final String FP = "tmsftp";
	private final String savePath = "/sdcard/Android/data/irvs.tms.tms_rs_ur_client/";
	private final String saveDir = "images";
	//=============================================

	//message

	private TextView result;
	private EditText run_time;
	//	private LinearLayout mainLL;
	//	private ViewFlipper vf;
	//	private TableLayout tl;
	private LinearLayout ll;
	//		private ScrollView sv;
	//	private Spinner sp;
	private Button btnRenew,btnSend,btnRoot;
	private GetAroundObject data;
	private GetObjectInfoRt info;
	//	ArrayList<Object> dummy;
	private FtpClient fclient;
	private ProgressDialog dialog;
	private static Object currentObject = null;
	//	private static ConstDraw cd = null;
	//	private static boolean drawing;
	private RenewSrv renewSrv;
	private static HandleDraw hd;

	public MainActivity() {
		super("rosandroid_demo_srv_client", "rosandroid_demo_srv_client");
	}

	@Override
	public void onPause(){
		super.onPause();
		//		cd = null;
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);

		result = (TextView)findViewById(R.id.txtResult);
		//		sv = (ScrollView)findViewById(R.id.scrollView1);
		ll = (LinearLayout)findViewById(R.id.objectLayout);
		//		tl = (TableLayout)findViewById(R.id.objectLayout);
		//		vf = (ViewFlipper)findViewById(R.id.flipper);
		//		sp = (Spinner)findViewById(R.id.spinnerPlace);
		//		usr_id = (EditText)findViewById(R.id.userID);
		run_time = (EditText)findViewById(R.id.timeID);
		btnRenew = (Button)findViewById(R.id.btnRenew);
		btnSend = (Button)findViewById(R.id.btnSend);
		btnRoot = (Button)findViewById(R.id.btnRoot);
		
		//		drawing = true;

		//		data = new GetTmsDbData();

		//dummy object
		//				dummy = new ArrayList<Object>();
		//		
		//		
		//		
		//		
		//				for(int i=0;i<15;i++){
		//					dummy.add(new Object(new String("Object" + i),i));
		//					dummy.get(i).setPosition(0f, 0f, 0f, 0f);
		//				}
		//				dummy.get(0).setSrcObject(dummy.get(3));
		//				dummy.get(0).setType(0);
		//				dummy.get(3).addDstObject(dummy.get(0));
		//				for(int i=1;i<15;i++){
		//					dummy.get(i).setSrcObject(dummy.get(0));
		//				}
		//				for(int i=1;i<15;i++){
		//					dummy.get(0).addDstObject(dummy.get(i));
		//					dummy.get(i).setType(5);
		//				}

		//		final ArrayAdapter<String> plist = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item);

		//Setting関連==================================
		loadPref("USERID","FTPIP","FTPUSR","FTPPASS");
		btnSetting = (Button)findViewById(R.id.btnSetting);
		//=============================================

		btnSetting.setOnClickListener(new OnClickListener(){
			@Override
			public void onClick(android.view.View v){
				LayoutInflater inflater 
				= LayoutInflater.from(MainActivity.this);
				View view = inflater.inflate(R.layout.setting, null);

				//SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(MainActivity.this);

				final EditText uid = (EditText)view.findViewById(R.id.editUID);
				final EditText fip = (EditText)view.findViewById(R.id.editFtpIP);
				final EditText fu = (EditText)view.findViewById(R.id.editFtpUsr);
				final EditText fp = (EditText)view.findViewById(R.id.editFtpPass);

				uid.setText(String.valueOf(USERID), BufferType.NORMAL);
				fip.setText(FTPIP, BufferType.NORMAL);
				fu.setText(FTPUSR, BufferType.NORMAL);
				fp.setText(FTPPASS, BufferType.NORMAL);

				Log.v("POPUP","Start!!");

				new AlertDialog.Builder(MainActivity.this)
				.setTitle("SETTING")
				.setIcon(R.drawable.setting2)
				.setView(view)
				.setPositiveButton(
						"APPLY", 
						new DialogInterface.OnClickListener() {          
							@Override
							public void onClick(DialogInterface dialog, int which) {
								USERID = Integer.valueOf(uid.getText().toString());
								FTPIP = fip.getText().toString();
								FTPUSR = fu.getText().toString();
								FTPPASS = fp.getText().toString();
								savePref("USERID","FTPIP","FTPUSR","FTPPASS");
								Log.v("POPUP","positive***");
								Log.v("POPUP","USERID:" + USERID);
								Log.v("POPUP","FTPIP:" + FTPIP);
								Log.v("POPUP","FTPUSR:" + FTPUSR);
								Log.v("POPUP","FTPPASS:" + FTPPASS);
							}
						})
						.setNeutralButton("DEFAULT", new DialogInterface.OnClickListener() {          
							@Override
							public void onClick(DialogInterface dialog, int which) {
								USERID = UID;
								FTPIP = FIP;
								FTPUSR = FU;
								FTPPASS = FP;
								savePref("USERID","FTPIP","FTPUSR","FTPPASS");
							}
						})
						.setNegativeButton("CANCEL",null)
						.show();
			}
		});

		btnRenew.setOnClickListener(new OnClickListener(){
			@Override
			public void onClick(android.view.View v){
				dialog = new ProgressDialog(MainActivity.this);
				dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
				dialog.setMessage("Download Image Files ...");
				dialog.show();
				//無理やり非同期処理
				AsyncTask<Void,Void,Void> FTPTask = new AsyncTask<Void,Void,Void>(){
					@Override
					protected Void doInBackground(Void... params){
						fclient = new FtpClient(FTPIP,FTPUSR,FTPPASS);
						fclient.ChangeDir("2D");//2Dフォルダへ
						fclient.SetDLPass(savePath,saveDir);//DLフォルダ作成
						fclient.Refresh();//DLフォルダの中身を削除
						fclient.DownloadAll();//imageフォルダの中身を全てダウンロード
						fclient.close();//解放(ぶっちゃけいらない)
						return null;
					}
					@Override
					protected void onPostExecute(Void result){
						dialog.dismiss();
						//viewの更新
					}
				};
				//非同期処理実行
				FTPTask.execute();
			}
		});

		btnSend.setOnClickListener(new OnClickListener(){
			@Override
			public void onClick(android.view.View v){

				//				drawWindow(data.obj);
				//				drawWindow(dummy.get(0));
				currentObject = null;
				currentObject = new Object("floor",11);

				try {
					//オブジェクト生成時間稼ぎ（適当）
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO 自動生成された catch ブロック
					e.printStackTrace();
				}
				drawWindow();
				//				if(cd == null) cd = new ConstDraw();
				//				cd.sleep(0);
				//				drawing = false;
			}
		});

		btnRoot.setOnClickListener(new OnClickListener(){
			@Override
			public void onClick(android.view.View v){
				currentObject = null;
				currentObject = new Object("floor",11);
				drawWindow();
			}
		});

		//drawWindow(new Object("floor",11));

	}


	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		//		srvClient = new SmartPal_Srv();
		////		srvClient2 = new SrvClient();
		data = new GetAroundObject();
		NodeConfiguration nodeConfiguration 
		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());

		nodeConfiguration.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(data, nodeConfiguration);  


		info = new GetObjectInfoRt();
		NodeConfiguration nodeConfiguration2 
		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());

		nodeConfiguration2.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(info, nodeConfiguration2);

		renewSrv = new RenewSrv();
		NodeConfiguration nodeConfiguration3 
		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());
		nodeConfiguration.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(renewSrv, nodeConfiguration3);
		
		hd = new HandleDraw();
		hd.sleep(0);

	}
	//画面の描画
	private void drawWindow(){
		//		drawing = true;
		if(currentObject == null){
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO 自動生成された catch ブロック
				e.printStackTrace();
			}
		}
		else{
			data.sendID(currentObject.getId());
			Button mainObj = (Button)findViewById(R.id.btnCurrentObj);
			mainObj.setText(currentObject.getName());
			mainObj.setOnClickListener(new OnClickListener(){
				@Override
				public void onClick(android.view.View v){
					info.sendID(currentObject.getId());
					popDatail(info.obj);
					//					result.setText(new String(obj.getName() + "\nUsrId:" + usr_id + "\nPlace:" + sp.getSelectedItem().toString() + "\nTime:" + run_time));
				}
			});

			ImageButton srcObj = (ImageButton)findViewById(R.id.btnReturn);
			srcObj.setOnClickListener(new OnClickListener(){
				@Override
				public void onClick(android.view.View v){
					//					currentObject.setName(data.src.getName());
					//					currentObject.setId(data.src.getId());
					currentObject.reset(data.src.getName(),data.src.getId());
					drawWindow();
				}
			});

			ll.removeAllViews();
			for(final Object object : data.obj){
				LinearLayout dstObj = (LinearLayout)getLayoutInflater().inflate(R.layout.objlayout,null);
				//				TableRow dstObj = (TableRow)getLayoutInflater().inflate(R.layout.objlayout,null);
				ImageView img = (ImageView)getLayoutInflater().inflate(R.layout.buttonicon,null);
				//				TextView objid = (TextView)getLayoutInflater().inflate(R.layout.txtid,null);
				//				TextView objname = (TextView)getLayoutInflater().inflate(R.layout.txtname,null);

				//iconのセット****************************************************
				Bitmap imgbitmap = null;
				String imname = String.valueOf(object.getId()) + ".JPG";
				File fimage = new File(savePath + saveDir,imname);
				InputStream is;
				try {
					is = new FileInputStream(fimage);
					imgbitmap = BitmapFactory.decodeStream(is);
					is.close();
				} catch (FileNotFoundException e) {
					// TODO 自動生成された catch ブロック
					imgbitmap = null;
					e.printStackTrace();
				} catch (IOException e) {
					// TODO 自動生成された catch ブロック
					e.printStackTrace();
				}

				if(imgbitmap == null){
					Resources r = getResources();
					imgbitmap = BitmapFactory.decodeResource(r, R.drawable.noimagebig);
				}
				img.setImageBitmap(imgbitmap);
				//iconのセット****************************************************

				//				objid.setText(String.valueOf(object.getId()));
				//				objname.setText(object.getName());

				//				img.setOnClickListener(new OnClickListener(){
				//					@Override
				//					public void onClick(android.view.View v){
				//						if(object.getType()!=5){
				//							currentObject = null;
				//							currentObject = new Object(object.getName(),object.getId());
				//							drawWindow();
				//						}
				//						else{
				//							info.sendID(object.getId());
				//							popDatail(info.obj);
				//						}
				//					}
				//				});
				dstObj.addView(img);
				//				dstObj.addView(objid);
				//				dstObj.addView(objname);
				dstObj.setOnClickListener(new OnClickListener(){
					@Override
					public void onClick(android.view.View v){
						if(object.getType()!=5){
							currentObject.reset(object.getName(),object.getId());
							drawWindow();
						}
						else{
							info.sendID(object.getId());
							popDatail(info.obj);
						}
					}
				});
				ll.addView(dstObj);
			}
		}
		//		drawing = false;
	}


	//	public class ConstDraw extends Handler {
	//	    @Override
	//	    public void handleMessage(Message msg) {
	////	    	if(currentObject != null) drawWindow(currentObject);
	////	    	Log.v("ConstDraw","in!");
	////	    	if(cd != null) cd.sleep(3000);  //3.
	//	        drawWindow();
	//	        
	//	    }
	//	 
	//	    //スリープメソッド
	////	    public void sleep(long delayMills) {
	////	        //使用済みメッセージの削除
	////	        removeMessages(0);
	////	        sendMessageDelayed(obtainMessage(0),delayMills);  //4.
	////	    }
	//	}

	private void popDatail(Object object){
		//		drawing = true;
		//		cd = null;
		//		if(cd == null);
		LayoutInflater inflater 
		= LayoutInflater.from(MainActivity.this);
		View view = inflater.inflate(R.layout.popupdetail, null);

		ImageView img = (ImageView)view.findViewById(R.id.img);
		TextView objId = (TextView)view.findViewById(R.id.txtId);
		TextView objName = (TextView)view.findViewById(R.id.txtName);
		TextView objPlace = (TextView)view.findViewById(R.id.txtPlace);
		TextView objX = (TextView)view.findViewById(R.id.txtX);
		TextView objY = (TextView)view.findViewById(R.id.txtY);
		TextView objZ = (TextView)view.findViewById(R.id.txtZ);
		TextView objT = (TextView)view.findViewById(R.id.txtT);

		//imgのセット****************************************************
		Bitmap imgbitmap = null;
		String imname = String.valueOf(object.getId()) + ".JPG";
		File fimage = new File(savePath + saveDir,imname);
		InputStream is;
		try {
			is = new FileInputStream(fimage);
			imgbitmap = BitmapFactory.decodeStream(is);
			is.close();
		} catch (FileNotFoundException e) {
			// TODO 自動生成された catch ブロック
			e.printStackTrace();
		} catch (IOException e) {
			// TODO 自動生成された catch ブロック
			e.printStackTrace();
		}

		if(imgbitmap != null) img.setImageBitmap(imgbitmap);
		//imgのセット****************************************************

		img.setImageBitmap(imgbitmap);
		objId.setText(String.valueOf(object.getId()));
		objName.setText(object.getName());
		objPlace.setText(object.getSrcName());
		objX.setText(String.valueOf(object.getPosition().getX()));
		objY.setText(String.valueOf(object.getPosition().getY()));
		objZ.setText(String.valueOf(object.getPosition().getZ()));
		objT.setText(String.valueOf(object.getPosition().getTheta()));



		Log.v("POPUP","Start!!");

		new AlertDialog.Builder(MainActivity.this)
		.setTitle("DETAIL")
		.setIcon(R.drawable.ic_launcher)
		.setView(view)
		.setPositiveButton(
				"GETOBJECT", 
				new DialogInterface.OnClickListener() {          
					@Override
					public void onClick(DialogInterface dialog, int which) {

					}
				})
				.setNegativeButton("CANCEL",new DialogInterface.OnClickListener() {          
					@Override
					public void onClick(DialogInterface dialog, int which) {

					}})
					.show();
		//		if(cd == null) cd = new ConstDraw();
		//		cd.sleep(0);
		//		drawing = false;
	}


	//プリファレンスの取得・保存
	private void savePref(String taguid,String tagfip, String tagfu, String tagfp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		pref.edit().putInt(taguid,USERID).commit();
		pref.edit().putString(tagfip,String.valueOf(FTPIP)).commit();
		pref.edit().putString(tagfu,String.valueOf(FTPUSR)).commit();
		pref.edit().putString(tagfp,String.valueOf(FTPPASS)).commit();
	}

	private void loadPref(String taguid,String tagfip, String tagfu, String tagfp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		USERID = pref.getInt(taguid, UID);
		FTPIP = pref.getString(tagfip, FIP);
		FTPUSR = pref.getString(tagfu, FU);
		FTPPASS = pref.getString(tagfp, FP);
	}

	//更新情報のidが一致していた場合のみ再描画
	public class HandleDraw extends Handler {
		@Override
		public void handleMessage(Message msg) {
			//	    	if(currentObject != null) drawWindow(currentObject);
			//	    	Log.v("ConstDraw","in!");
			//	    	if(cd != null) cd.sleep(3000);  //3.
			if(hd != null) hd.sleep(3000);
			if(renewSrv.getFlag()
					&&(Integer.valueOf(currentObject.getId())
							.equals(renewSrv.getRenewId())
							||renewSrv.getRenewId()==0)) drawWindow();
			renewSrv.clearFlag();

		}

		//スリープメソッド
			    public void sleep(long delayMills) {
			        //使用済みメッセージの削除
			        removeMessages(0);
			        sendMessageDelayed(obtainMessage(0),delayMills);  //4.
			    }
	}

}
