package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.os.Handler;
import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

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
        try {
            rpClient = connectedNode.newServiceClient("hogehogehoge", rp_cmd._TYPE);
        } catch (ServiceNotFoundException e) {
            Log.d(TAG, "ServiceNotFoundException");
            //throw new RosRuntimeException(e);
        }
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);
    }

    public void sendRequest(float x, float y, float z, float theta) {
        final rp_cmdRequest req = rpClient.newMessage();

        final double[] arg = {-1, (double)x, (double)y, (double)theta};
        req.setCommand(9001); // move
        req.setType(true); // real (not simulation mode)
        req.setRobotId(2007); // Mimamorukun
        req.setArg(arg); // several arguments

        rpClient.call(req, new ServiceResponseListener<rp_cmdResponse>() {
            @Override
            public void onSuccess(rp_cmdResponse res) {
                Log.d(TAG, "onSuccess()" + res.getResult());
            }

            @Override
            public void onFailure(RemoteException e) {
                Log.d(TAG, "onFailure");
            }
        });
    }
}
