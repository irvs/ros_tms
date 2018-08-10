const express = require("express");
//const ipfilter = require("express-ipfilter").IpFilter;
const bodyParser = require("body-parser");
const app = express();
const request = require("request");
const url = require('url');
const ipware = require('ipware');
const ipw = ipware();

const fs = require("fs");
const url_path = "resources/tms.json";

let tms_reciever_port = ":3000/post";

const tms_list=["hoge"];

function getUrlList(filepath){
    let urlList = JSON.parse(fs.readFileSync(filepath, 'utf8'));
    urlList.tms.forEach(data => {
        tms_list.push(data.url);
    });
    console.log(tms_list);
}

getUrlList(url_path);

console.log(tms_list);


/*TO DO 
    this list should be get from outside resources such as DB
*/
//app.use(tms_list.forEach((tms_url) => ipfilter(tms_url, {mode: 'allow'})));
app.use(bodyParser.json());
//app.set('trust proxy', true);

app.post("/rms_svr", (req, res) => {
    let tms_sender= "http://";
    //console.log(ipw.get_ip(req));
    const tms_sender_ip = ((ipw.get_ip(req).clientIp).split(":")).pop();
    //console.log(tms_sender_ip);
    tms_sender += tms_sender_ip; 
    let tmp_tms_list = [];
    console.log("Http request from: " + tms_sender);
    const service = req.body;

    
    if(tms_list.indexOf(tms_sender) == -1){
        res.json({
            "message":"This host has no authority to access this server"
        });
        return;
    }else{
        tmp_tms_list = tms_list.filter((val) => {
            if(val != tms_sender){
                return val;
            }
        })
    }

    //console.log(`[${new Date()}] request = [${JSON.stringify(service)}]`);
/*
    let robot_name = service.robot;
    let task_name = service.task;
    let user_name = service.user;
    let object_name = service.object;
    let place_name = service.place;
*/


    
/*
    const update_url = (new_url) =>{
        tms_reciever = new_url;
        //console.log(tms_reciever);
    };
*/
    
    const search_tms = (tms_url) => {
        return new Promise((resolve, reject) =>{
            const headers = {
                'Content-Type':'application/json'
            }
            const options = {
                url: tms_url,
                //method: 'POST',
                headers: headers,
                json: {
                    "robot": service.robot,
                    "task": service.task,
                    "user": service.user,
                    "object": service.object,
                    "place":service.place
                }
            }

            request.post(options, function(error, response, body){
                if(error){
                    //return false;
                    //reject(new Error("Could not connect to Host"));
                    resolve(0);
                    return false;
                }
                if (!error && response.body.message == "OK"){
                        //update_url(response.request.uri.href);
                        //console.log(tms_url); 
                        console.log(response.body);
                        const response_obj = {
                            "message":"OK",
                            "hostname":response.request.uri.hostname,
                            "service_id":{
                                "robot_id":response.body.service_id.robot_id,
                                "task_id":response.body.service_id.task_id,
                                "user_id":response.body.service_id.user_id,
                                "object_id":response.body.service_id.object_id,
                                "place_id":response.body.service_id.place_id,   
                            }
                        }
                        resolve(response_obj);
                        return true;
                }else{
                    console.log('error: '+response.body.message +" @ "+ tms_url);
                    //return false;
                    resolve(0);
                    return false;
                    //reject(new Error(response.body.message));
                }
            });
        });
    };
    
//    search_tms(tms_reciever);
    
    let promises = [];
    for(let i = 0; i < tmp_tms_list.length; i++){
        promises.push(search_tms(tmp_tms_list[i] + tms_reciever_port));
    }
    Promise.all(promises).then(function(results){
        for(let i = 0; i < tmp_tms_list.length; i++){
            if(!results[i] == 0){
                res.json(results[i]);
                break;
            }else if(i == tmp_tms_list.length - 1){
                res.json({"message":"Could not find them in all tms"});
            }
        }
    })


});

app.listen(4000);