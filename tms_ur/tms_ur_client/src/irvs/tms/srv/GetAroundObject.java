/*
 *sendID(object_id)を呼び出せばID＝object_idに所属するObjectの名前とIDのリストを返す
 */

package irvs.tms.srv;

import java.util.ArrayList;

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
import tms_msg_ur.tms_rs_get_around_object;
import tms_msg_ur.tms_rs_get_around_objectRequest;
import tms_msg_ur.tms_rs_get_around_objectResponse;


public class GetAroundObject extends AbstractNodeMain {

	ServiceClient<tms_rs_get_around_objectRequest, tms_rs_get_around_objectResponse> SrviceClient;

	int  isCalled=0;
	int result;
	public ArrayList<Object> obj;
	public Object src;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rs_get_object_srv_client/GetAroundObject");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		try {
			SrviceClient = connectedNode.newServiceClient("tms_rs_get_around_object",tms_rs_get_around_object._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}   
	}
	public int sendID(int req){
		Log.v("sendId", "start");
		int cnt = 0;
		result = 0;
		isCalled = 0;
		tms_rs_get_around_objectRequest request = SrviceClient.newMessage();

		request.setSrcObjectId(req);
		SrviceClient.call(request, new ServiceResponseListener<tms_rs_get_around_objectResponse>(){
			@Override
			public void onSuccess(tms_rs_get_around_objectResponse response) {
				Log.v("sendId", "onSuccess");
				isCalled = 1;
				obj = new ArrayList<Object>();
				src = new Object(response.getSrcName(),response.getSrcId());
//				obj.add(new Object("ROOT",1000));

				//src dst以外の情報を追加
				Log.v("Len",String.valueOf(response.getObjectId().length));
				for(int i=0;i<response.getObjectId().length;i++){
					obj.add(new Object(response.getName().get(i), response.getObjectId()[i],response.getType()[i],1));
				}
				
				Log.v("sendId", "isCalled");
				result = obj.size();
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
