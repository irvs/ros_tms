Bleacon = require('bleacon');
Bleacon.startScanning();

var dgram = require('dgram');
var client = dgram.createSocket('udp4');
const PORT = 9090;
const ADDR = "192.168.4.170";

const MY_ID = 1;

Bleacon.on('discover', function(bleacon) {
  bleacon["my_id"] = MY_ID;
  var message = new Buffer(JSON.stringify(bleacon));
  client.send(message,0,message.length,PORT,ADDR,function(err,bytes){
	if(err){
	   throw err;
	}
   });
   console.dir(bleacon);
});
