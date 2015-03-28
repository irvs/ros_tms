package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceClient;

import tms_msg_rp.rp_cmd;
import tms_msg_rp.rp_cmdRequest;
import tms_msg_rp.rp_cmdResponse;

/**
 * Created by kazuto on 15/03/19.
 */
public class rp_cmd_client extends AbstractNodeMain {
    private String TAG = "rp_cmd_client";
    private ServiceClient<rp_cmdRequest, rp_cmdResponse> rpClient;
    private Handler handler;

    public rp_cmd_client(Handler handler) {
        this.handler = handler;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tms_ur_mimamorukun/rp_cmd_client");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d(TAG, "onStart");
        super.onStart(connectedNode);
        String status;
        try {
            rpClient = connectedNode.newServiceClient("hogehogehoge", rp_cmd._TYPE);
            status = "connected";
        } catch (ServiceNotFoundException e) {
            Log.d(TAG, "ServiceNotFoundException");
            status = "failed to connect";
            //throw new RosRuntimeException(e);
        }
        Message msg = handler.obtainMessage(TmsUrMimamorukun.RP_CMD_STATUS, status);
        handler.sendMessage(msg);
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);
    }

    public void sendRequest(double x, double y, double yaw) {
        Log.d(TAG, "sendRequest()");
/*

        final rp_cmdRequest req = rpClient.newMessage();

        final double[] arg = {-1, x, y, yaw};
        req.setCommand(9001); // move
        req.setType(true); // real (not simulation mode)
        req.setRobotId(2007); // Mimamorukun
        req.setArg(arg); // several arguments

        rpClient.call(req, new ServiceResponseListener<rp_cmdResponse>() {
            @Override
            public void onSuccess(rp_cmdResponse res) {
                Log.d(TAG, "onSuccess()" + res.getResult());
                Message msg = handler.obtainMessage(TmsUrMimamorukun.RP_CMD_RESULT, "succeeded");
                handler.sendMessage(msg);
            }

            @Override
            public void onFailure(RemoteException e) {
                Log.d(TAG, "onFailure");
                Message msg = handler.obtainMessage(TmsUrMimamorukun.RP_CMD_RESULT, "Failed to connect the server");
                handler.sendMessage(msg);
            }
        });
*/
        Message msg = handler.obtainMessage(TmsUrMimamorukun.RP_CMD_RESULT, "debug");
        handler.sendMessage(msg);
    }
}
