/*
 *sendID(object_id)を呼び出せばID＝object_idに所属するObjectの名前とIDのリストを返す
 */

package irvs.tms.srv;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import android.util.Log;

import irvs.tms.data.Object;
import tms_msg_ur.tms_rs_get_object_info_rt;
import tms_msg_ur.tms_rs_get_object_info_rtRequest;
import tms_msg_ur.tms_rs_get_object_info_rtResponse;


public class GetObjectInfoRt extends AbstractNodeMain {

	ServiceClient<tms_rs_get_object_info_rtRequest, tms_rs_get_object_info_rtResponse> SrviceClient;

	int  isCalled=0;
	int result;
	public Object obj;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rs_get_object_srv_client/GetObjectInfoRt");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		try {
			SrviceClient = connectedNode.newServiceClient("tms_rs_get_object_info_rt",tms_rs_get_object_info_rt._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}   
	}
	public int sendID(int req){
		Log.v("sendId", "start");
		int cnt = 0;
		result = 0;
		isCalled = 0;
		tms_rs_get_object_info_rtRequest request = SrviceClient.newMessage();

		request.setRequest(req);
		SrviceClient.call(request, new ServiceResponseListener<tms_rs_get_object_info_rtResponse>(){
			@Override
			public void onSuccess(tms_rs_get_object_info_rtResponse response) {
				Log.v("sendId", "onSuccess");
				isCalled = 1;
				obj = new Object(response.getName(),response.getObjectId(),response.getType(),response.getState());
				obj.setPosition(response.getX(), response.getY(), response.getZ(), response.getTheta());
				obj.setSrc(response.getPlaceId(), response.getPlaceName());
				Log.v("sendId", "isCalled");
				result = obj.getId();
			}
			@Override
			public void onFailure(RemoteException e) {
				Log.v("sendId", "onFailure");
				throw new RosRuntimeException(e);
			}
		});
		Log.v("sendId", "wait");
		while(isCalled!=1 && cnt<10){  
			try {
				Thread.sleep(500);  // 500ms * 10 = 5sec
			} catch (InterruptedException e){}
			cnt++;
		}
		Log.v("sendId", "result");
		return result;
	}
}
