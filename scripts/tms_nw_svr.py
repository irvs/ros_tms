#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import requests
from BaseHTTPServer import HTTPServer
from BaseHTTPServer import BaseHTTPRequestHandler
import urlparse

def svr_start(port, callback):
    def handler(*args):
        CallbackServer(callback, *args)
    server = HTTPServer(('', int(port)), handler)
    server.serve_forever()

class tms_nw_svr(BaseHTTPRequestHandler):
    def __init__(self, callback, *args):
        self.callback = callback
        BaseHTTPRequestHandler.__init__(self, args)

    def do_GET(self):
        parsed_path = urlparse.urlparse(self.path)
        query = parsed_path.query
        self.send_response(200)
        self.end_headers()
        result = self.callback(query)
        message = '\r\n'.join(result)
        self.wfile.write(message)
        return