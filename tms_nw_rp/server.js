const setTimeout =  require("timers").setTimeout;
const roslib = require("roslib");
const http = require("http");
/*
roslib.Service.prototype._serviceResponse = (rosbridgeRequest) =>{
    let response = {};
    
    let promise = Promise.resolve();
    promise
        .then(() =>{
            return new Promise((resolve, reject) => {
                let success = this._serviceCallback(rosbridgeRequest.args, response);
                resolve(success);
            });
        }).then(() => {
            let call = {
                op: 'service_response',
                service: this.name,
                values: new ServiceResponse(response),
                result: success
            }
            if (rosbridgeRequest.id){
                call.id = rosbridgeRequest.id;
            }
            this.ros.callOnConnection(call);
        });

}
*/


const ros_local = new roslib.Ros({
    url:'ws://localhost:9090'
}); 

const req_server = new roslib.Service({
    ros: ros_local,
    name: '/tms_nw_req',
    serviceType: 'tms_nw_rp/tms_nw_req',
});

ros_local.on('connection', () => {
    console.log('Connected to websocket server');
});

ros_local.on('error', error =>{
    console.log('Erro connecting to websocket server: ', error);
});

ros_local.on('close', () => {
    console.log("Connection to websocket server was closed");
    req_server.unadvertise();
});

let req_service_type = 'hoge';



req_server.advertise(async (req, res) => {
    const ros_remote = new roslib.Ros({
        url: "ws://" + req.url + ":9090"
    });
    ros_remote.on('connection', () => {
        console.log('Connected to websocket server');
    });
    
    ros_remote.on('error', error =>{
        console.log('Erro connecting to websocket server: ', error);
    });
    
    ros_remote.on('close', () => {
        console.log("Connection to websocket server was closed");
    });
    res.result = 1;
    
    const remote_service = new roslib.Service({
        ros: ros_remote,
        name: req.service_name,
        serviceType: req.service_type,
    });

    delete req.service_name;
    delete req.service_type;
    delete req.url;
    //req.rostime = 0;

    let remote_req = new roslib.ServiceRequest(req);
    console.log(remote_req);

    remote_service.callService(remote_req, res_remote => {
        res = res_remote;
        console.log(res_remote);
    })
        

});

