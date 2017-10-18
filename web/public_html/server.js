var connect = require('connect');
var fs = require('fs');
var http = require('http');
var serveStatic = require('serve-static');
var express = require('express');

var app = express();

var xml2js = require("xml2js");
var parser = new xml2js.Parser();

var builder = require("xmlbuilder");

connect().use(serveStatic(__dirname)).listen(8080, function(){
    console.log('Server running on 8080...');
});

app.get("/getWillyConfig", function(req, res){
	fs.readFile("/home/hjleusink/Documents/WillyConfig.xml", function(error, data) {
		parser.parseString(data, function(error, result) {
			res.set("Content-Type", "text/json"); 
		  	res.set("Access-Control-Allow-Origin", "*");
			res.set("Access-Control-Allow-Headers", "X-Requested-With");

			res.send(result);
		});
	});
});

app.post('/setWillyConfig', function(request, result){
	var data = request.body;

	console.log(data);

	res.set("Content-Type", "text/json"); 
  	res.set("Access-Control-Allow-Origin", "*");
	res.set("Access-Control-Allow-Headers", "X-Requested-With");

	res.send(true);
});


app.listen(3000);