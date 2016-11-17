Bleacon = require('bleacon');
Bleacon.startScanning();

var dgram = require('dgram');
var client = dgram.createSocket('udp4');
const PORT = 9090;
const ADDR = "192.168.64.39";

Bleacon.on('discover', function(bleacon) {
   var message = new Buffer(JSON.stringify(bleacon));
   client.send(message,0,message.length,PORT,ADDR,function(err,bytes){
	if(err){
	   throw err;
	}
   });
   console.dir(bleacon);
});


