#!/usr/bin/python27
 
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.gen
from tornado.options import define, options
 
import os
import time
import multiprocessing
import serialProcess
import json
 
define("port", default=8080, help="run on the given port", type=int)

clients = []
 
class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html')
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print 'new connection'
        clients.append(self)
        # self.write_message("connected")
 
    def on_message(self, message):
        print 'tornado received from client: %s' % message
        # self.write_message('got it!')
        q = self.application.settings.get('queue')
        q.put(message)
 
    def on_close(self):
        print 'connection closed'
        clients.remove(self)
 
################################ MAIN ################################
 
def main():
 
    taskQ = multiprocessing.Queue()
    resultQ = multiprocessing.Queue()
 
    sp = serialProcess.SerialProcess(taskQ, resultQ)
    sp.daemon = True
    sp.start()
 
    # wait a second before sending first task
    time.sleep(1)
    taskQ.put("first task")
 
    tornado.options.parse_command_line()
    app = tornado.web.Application(
        handlers=[
            (r"/", IndexHandler),
            (r"/mazebuster", WebSocketHandler)
        ], queue=taskQ,
        static_path=os.path.join(os.path.dirname(__file__), 'static')
    )
    httpServer = tornado.httpserver.HTTPServer(app)
    httpServer.listen(options.port)
    print "Listening on port:", options.port
    #tornado.ioloop.IOLoop.instance().start()
 
    def checkResults():
        if not resultQ.empty():
            result = resultQ.get()
            if str(result).startswith('//'):
            	print "tornado received from arduino: " + str(result)
            for c in clients:
                c.write_message(result)
 
    mainLoop = tornado.ioloop.IOLoop.instance()
    scheduler = tornado.ioloop.PeriodicCallback(checkResults, 10, io_loop = mainLoop)
    scheduler.start()
    mainLoop.start()
 
if __name__ == "__main__":
    main()