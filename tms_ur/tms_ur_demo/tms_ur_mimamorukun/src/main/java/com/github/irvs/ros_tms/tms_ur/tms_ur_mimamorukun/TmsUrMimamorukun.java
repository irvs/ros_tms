package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.content.res.AssetManager;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.support.v4.app.ActionBarDrawerToggle;
import android.support.v4.widget.DrawerLayout;
import android.util.Log;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.SeekBar;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.io.InputStream;
import java.net.URI;

public class TmsUrMimamorukun extends RosActivity
{
    private String TAG = "tms_ur_mimamorukun";
    private TextView touch_position, touch_action, touch_time;
    private ImageView wc_icon;
    private ImageView map_image;
    private float[] target_point = {0, 0};
    private float[] wc_icon_size = {0, 0};
    private SeekBar seekBar;
    private int mode = 1;
    private ActionBarDrawerToggle drawerToggle;
    private DrawerLayout drawer;
    private RelativeLayout main_layout;

    // ros nodes
    private rp_cmd_client rp_cmd_client;
    private db_reader_client db_reader_client;

    public TmsUrMimamorukun()
    {
        super("Mimamorukun Control", "Mimamorukun Control", URI.create("http://192.168.4.170:11311"));
    }

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        main_layout = (RelativeLayout)findViewById(R.id.main_layout);

        // for debug
        touch_position = (TextView)findViewById(R.id.touch_position);
        touch_action = (TextView)findViewById(R.id.touch_action);

        // the icon of a wheelchair & the map
        wc_icon = (ImageView)findViewById(R.id.wc_icon);
        map_image = (ImageView)findViewById(R.id.map_image);
        AssetManager as = getResources().getAssets();
        try {
            InputStream is = as.open("images/wc_icon.png");
            Bitmap bm = BitmapFactory.decodeStream(is);
            wc_icon.setImageBitmap(bm);

        } catch (IOException e){
            Log.e(TAG, e.toString());
        }
        try {
            InputStream is = as.open("images/map_image.png");
            Bitmap bm = BitmapFactory.decodeStream(is);
            map_image.setImageBitmap(bm);
        } catch (IOException e) {
            Log.e(TAG, e.toString());
        }
        wc_icon_size[0] = wc_icon.getDrawable().getIntrinsicWidth();
        wc_icon_size[1] = wc_icon.getDrawable().getIntrinsicHeight();
        wc_icon.setVisibility(View.INVISIBLE);

        // to adjust the wc's orientation
        seekBar = (SeekBar)findViewById(R.id.seekBar);
        seekBar.setMax(360);
        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                wc_icon.setRotation(progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                ;
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                ;
            }
        });

        // navigation drawer
        drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
        drawerToggle = new ActionBarDrawerToggle(this, drawer,
            org.ros.android.android_10.R.mipmap.icon, R.string.drawer_open,
            R.string.drawer_close) {
            @Override
            public void onDrawerClosed(View drawerView) {
                Log.d(TAG, "onDrawerClosed");
            }

            @Override
            public void onDrawerOpened(View drawerView) {
                Log.d(TAG, "onDrawerOpened");
            }

            @Override
            public void onDrawerSlide(View drawerView, float slideOffset) {
                super.onDrawerSlide(drawerView, slideOffset);
                drawer.bringToFront();
                drawer.requestLayout();
                Log.d(TAG, "onDrawerSlide : " + slideOffset);
            }

            @Override
            public void onDrawerStateChanged(int newState) {
                // 表示済み、閉じ済みの状態：0
                // ドラッグ中状態:1
                // ドラッグを放した後のアニメーション中：2
                Log.i(TAG, "onDrawerStateChanged  new state : " + newState);
            }
        };

        drawer.setDrawerListener(drawerToggle);
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
        drawerToggle.syncState();
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        drawerToggle.onConfigurationChanged(newConfig);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // ActionBarDrawerToggleにandroid.id.home(up ナビゲーション)を渡す。
        if (drawerToggle.onOptionsItemSelected(item)) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void onStart() {
        super.onStart();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration
            = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        rp_cmd_client = new rp_cmd_client();
        nodeMainExecutor.execute(rp_cmd_client, nodeConfiguration);

        db_reader_client = new db_reader_client();
        nodeMainExecutor.execute(db_reader_client, nodeConfiguration);
    }

    @Override
    public boolean dispatchTouchEvent(MotionEvent event) {
        Log.d(TAG, "X:" + event.getX() + ",Y:" + event.getY());
        target_point[0] = event.getX();
        target_point[1] = event.getY();
        touch_position.setText("X:" + event.getX() + ", Y:" + event.getY());

        if (mode == 1) {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    Log.d(TAG, "getAction()" + "ACTION_DOWN");
                    touch_action.setText("ACTION_DOWN");
                    drawTarget();
                    break;
                case MotionEvent.ACTION_UP:
                    Log.d(TAG, "getAction()" + "ACTION_UP");
                    touch_action.setText("ACTION_UP");
                    break;
                case MotionEvent.ACTION_MOVE:
                    Log.d(TAG, "getAction()" + "ACTION_MOVE");
                    touch_action.setText("ACTION_MOVE");
                    drawTarget();
                    break;
                case MotionEvent.ACTION_CANCEL:
                    Log.d(TAG, "getAction()" + "ACTION_CANCEL");
                    touch_action.setText("ACTION_CANCEL");
                    break;
            }
        } else {
            ;
        }

        return super.dispatchTouchEvent(event);
    }

    public boolean drawTarget() {
        wc_icon.setVisibility(View.VISIBLE);
        wc_icon.setX(target_point[0] - wc_icon_size[0] / 2);
        wc_icon.setY(target_point[1] - wc_icon_size[1] / 2);
        return true;
    }


    public boolean drawCurrentPosition() {

        return true;
    }
}
