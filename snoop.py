# Python modules
import time
import sys
import os
import sysv_ipc
import requests
import threading 
import Queue
import subprocess

from requests.auth import HTTPBasicAuth


class WebThread(threading.Thread):
    """ A worker thread that takes takes commands to 
        upload a file to the web server or ping the web server.
    """
    def __init__(self, input_q, output_q, unit_id, host):
        super(WebThread, self).__init__()
        self.input_q = input_q
        self.output_q = output_q
        self.unit_id = unit_id
        self.host = host
        self.curr_event_time = 0

    def run(self):
        while 1:
            msg = self.input_q.get()
            cmd = msg[0]
            print "CMD: " + cmd
            if (cmd == "PING"):
                self.ping()
            elif (cmd == "UPLOAD"):
                self.upload(msg[1])
            elif (cmd == "EXIT"):
                break
            else:
                print "Unknown cmd:" , cmd
        print "WebThread exiting"

    def upload(self, filename):
        """ Upload specified file to web server
        """
        pieces = filename.split("/")
        dest_filename = pieces[2]
        pieces = dest_filename.split("_")
        file_time = pieces[0]
        curr_time = int(file_time)
        if ((self.curr_event_time == 0) or ((curr_time - self.curr_event_time) > 30)):
            self.curr_event_time = curr_time
       
        url = "http://"+self.host+"/snoop/events/upload/"+self.unit_id+"/"+str(self.curr_event_time)
        print url
        files = {'file': (dest_filename, open(filename, 'rb'))}
        r = requests.post(url, files=files, auth=HTTPBasicAuth('hambtw', 'Snoop123'))
        print r.status_code
        #print r.headers
        print r.text
        print "Removing: " + filename
        os.remove(filename)
       
    def ping(self):
        """ Ping the web server of this unit
        """
        url = "http://"+self.host+"/snoop/unitping/"+self.unit_id
        print url
        r = requests.get(url, auth=HTTPBasicAuth('hambtw', 'Snoop123'))
        print r.status_code
        #print r.headers
        print r.text

class MsgQueueThread(threading.Thread):
    """ A worker thread that reads the SYSV message queue
    """
    def __init__(self, sysv_msg_q, output_q):
        super(MsgQueueThread, self).__init__()
        self.sysv_msg_q = sysv_msg_q
        self.output_q = output_q

    def run(self):
        while 1:
            s,t = self.sysv_msg_q.receive(True, 1)
            s = s.decode()
            print("SYSV Msg Received: %s" %  s)
            if (s == "EXIT"):
                break
            else:
                self.output_q.put(("UPLOAD",s));
        print "MsgQueueThread exiting"

def main(args):
    # Create a single input and a single output queue for all threads.
    unit_id = "1"
    host = "snoop-wileycoyote.rhcloud.com"
    # Create Thread Queues
    web_q = Queue.Queue()
    my_q = Queue.Queue()
    # Create the SYSV message queue.
    mq = sysv_ipc.MessageQueue(500, sysv_ipc.IPC_CREAT, 0644)
    web_thread = WebThread(web_q, my_q, unit_id, host)
    web_thread.start()
    mq_thread = MsgQueueThread(mq, web_q)
    mq_thread.start()

    args = ["/opt/snoop/snoopmon"]
    proc = subprocess.Popen(args)

    while 1:
        try:
            params = my_q.get(True, 5.0)
            print params
            # mq.send(msg, 2)
            # Send to subproces via sysv queue
        except Queue.Empty:
            if (proc.poll() is not None):
                print "Subprocess Terminated!, code = ", proc.returncode
                break;
            else:
                web_q.put(("PING", ""))    

    web_q.put(("EXIT", ""))    
    mq.send("EXIT", 1)
    time.sleep(1)
    print("Destroying the message queue.")
    mq.remove()

if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
    print "main complete"
    quit()
