package irvs.tms.tms_ur.tms_ur_demo.tms_ur_demo_client;

import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tms_msg_rs.rs_home_appliancesRequest;
import tms_msg_rs.rs_home_appliancesResponse;

/**
 * RecognitionClient node
 * Receiving the recognized result, sends requests to the ROS service.
 */
public class RecognitionClient extends AbstractNodeMain {
    ServiceClient<rs_home_appliancesRequest, rs_home_appliancesResponse> serviceClient;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("RecognitionClient");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        /**
         * Connect to server with the service, "ref_demo".
         * "rs_home_appliances.srv"
         * --------------------------------------------
         * int32 id         // Refrigerator ID: 2009
         * int32 service    // 0: Close, 1: Open
         * ---
         * int32 result     // 0: failure, 1: Success
         * --------------------------------------------
         */
        try {
            serviceClient = connectedNode.newServiceClient("ref_demo", tms_msg_rs.rs_home_appliances._TYPE);
        } catch (ServiceNotFoundException e) {
            try {
                serviceClient = connectedNode.newServiceClient("ref_demo", tms_msg_rs.rs_home_appliances._TYPE);
            } catch (ServiceNotFoundException e2) {
                try {
                    serviceClient = connectedNode.newServiceClient("ref_demo", tms_msg_rs.rs_home_appliances._TYPE);
                } catch (ServiceNotFoundException e3) {
                    //throw new RosRuntimeException(e3);
                }
            }
        }
    }

    /**
     * Sends the recognized text to server
     */
    public void sendRequest(int num) {
        final tms_msg_rs.rs_home_appliancesRequest request = serviceClient.newMessage();

        /**
         * Set the parameter of the service
         */
        request.setId(2009);
        request.setService(num);

        /**
         * Call the service, "ref_demo".
         */
        Log.i("RecognitionClient","Call service");
        serviceClient.call(request, new ServiceResponseListener<rs_home_appliancesResponse>() {
            @Override
            public void onSuccess(final tms_msg_rs.rs_home_appliancesResponse srvResponse) {
                Log.i("RecognitionClient", "Succeeded to call service");
            }
            @Override
            public void onFailure(RemoteException e) {
                Log.i("RecognitionClient", "Failed to call service");
                throw new RosRuntimeException(e);
            }
        });
    }

}