package irvs.tms.srv;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;

import tms_msg_ur.tms_rs_renew;
import tms_msg_ur.tms_rs_renewRequest;
import tms_msg_ur.tms_rs_renewResponse;

//サービスから更新
public class RenewSrv extends AbstractNodeMain {

	private int renewId;
	private boolean renewFlag;

	public int getRenewId(){
		return renewId;
	}

	public boolean getFlag(){
		return renewFlag;
	}

	public void clearFlag(){
		renewFlag = false;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("tms_rs_ur_client/RenewSrv");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		connectedNode.newServiceServer("tms_rs_renew", tms_rs_renew._TYPE,
				new ServiceResponseBuilder<tms_rs_renewRequest, tms_rs_renewResponse>() {
					@Override
					public void
					build(tms_rs_renewRequest request, tms_rs_renewResponse response) {
						renewId = request.getId();
						renewFlag = true;
					}
				});
	}
}