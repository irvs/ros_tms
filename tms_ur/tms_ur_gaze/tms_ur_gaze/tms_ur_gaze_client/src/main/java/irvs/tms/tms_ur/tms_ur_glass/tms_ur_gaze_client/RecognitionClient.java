package irvs.tms.tms_ur.tms_ur_glass.tms_ur_gaze_client;

import android.os.Handler;
import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tms_ur_gaze_server.recognized_textRequest;
import tms_ur_gaze_server.recognized_textResponse;

/**
 * Created by kazuto on 9/18/14.
 *
 */
public class RecognitionClient extends AbstractNodeMain {
    ServiceClient<recognized_textRequest, recognized_textResponse> serviceClient;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("RecognitionClient");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        Log.i("ROS:Recognition Client", "onStart()");
        /* try to connect some times just to be sure */
        try {
            serviceClient = connectedNode.newServiceClient("recognition_test", tms_ur_gaze_server.recognized_text._TYPE);
        } catch (ServiceNotFoundException e) {
            try {
                serviceClient = connectedNode.newServiceClient("recognition_test", tms_ur_gaze_server.recognized_text._TYPE);
            } catch (ServiceNotFoundException e2) {
                try {
                    serviceClient = connectedNode.newServiceClient("recognition_test", tms_ur_gaze_server.recognized_text._TYPE);
                } catch (ServiceNotFoundException e3) {
                    throw new RosRuntimeException(e3);
                }
            }
        }
    }

    /**
     * Sends the recognized text to server
     */
    public void sendText(final String string) {
        final Handler handler = new Handler();

        final tms_ur_gaze_server.recognized_textRequest request = serviceClient.newMessage();
        request.setRequest(string);

        Log.i("ROS:Recognition Client","Call service with " + string);
        serviceClient.call(request, new ServiceResponseListener<recognized_textResponse>() {
            @Override
            public void onSuccess(final tms_ur_gaze_server.recognized_textResponse srvResponse) {
                Log.i("ROS:Recognition Client", "Succeeded to call service");
                handler.post(new Runnable(){
                    @Override
                    public void run() {
                        //TmsUrGazeClient.guiResult.setText(srvResponse.getResponse());
                        TmsUrGazeClient.guiResult.setText(string);
                        Log.i("ROS:Recognition Client", srvResponse.getResponse());
                    }
                });
            }
            @Override
            public void onFailure(RemoteException e) {
                Log.i("ROS:Recognition Client", "Failed to call service");
                throw new RosRuntimeException(e);
            }
        });
    }

}