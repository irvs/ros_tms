package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tms_msg_db.TmsdbGetData;
import tms_msg_db.TmsdbGetDataRequest;
import tms_msg_db.TmsdbGetDataResponse;

/**
 * Created by kazuto on 15/03/24.
 */
public class db_reader_client extends AbstractNodeMain {
    private String TAG = "db_reader_client";
    private ServiceClient<TmsdbGetDataRequest, TmsdbGetDataResponse> dbClient;
    private Handler handler;
    public Pose current_pose = new Pose();

    public db_reader_client(Handler handler) {
        this.handler = handler;
    }

    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tms_ur_mimamorukun/db_reader_client");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d(TAG, "onStart");
        String status;
        try {
            dbClient = connectedNode.newServiceClient("/tms_db_reader/dbreader", TmsdbGetData._TYPE);
            status = "Connected";
        } catch (ServiceNotFoundException e) {
            Log.d(TAG, "ServiceNotFoundException");
            status = "Failed to connect";
            //throw new RosRuntimeException(e);
        }
        Message msg = handler.obtainMessage(TmsUrMimamorukun.DBREADER_STATUS, status);
        handler.sendMessage(msg);

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            final TmsdbGetDataRequest req = dbClient.newMessage();

            @Override
            protected void setup() {
                Log.d(TAG, "setup");
                req.getTmsdb().setId(2007); // Mimamorukun ID
                req.getTmsdb().setSensor(3001); // 3501 kalman filter data
            }

            @Override
            protected void loop() throws InterruptedException {
                Log.d(TAG, "loop()");
                dbClient.call(req, new ServiceResponseListener<TmsdbGetDataResponse>() {
                    @Override
                    public void onSuccess(TmsdbGetDataResponse res) {
                        Log.d(TAG, "onSuccess()");
                        current_pose.x = res.getTmsdb().get(0).getX();
                        current_pose.y = res.getTmsdb().get(0).getY();
                        current_pose.yaw = res.getTmsdb().get(0).getRy();
                        Message msg = handler.obtainMessage(TmsUrMimamorukun.UPDATE_POSITION,
                            "from db_reader\n\tX:" + current_pose.x + ",Y:" + current_pose.y + ",Yaw:" + current_pose.yaw);
                        handler.sendMessage(msg);
                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        Log.d(TAG, "onFailure");
                    }
                });

                // update the position on regular basis
                Thread.sleep(500); // 500msec
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);
    }
}
