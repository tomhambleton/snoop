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

    def upload(self, filepath):
        """ Upload specified file to web server
        """
        """Assume /tmp/<filename>.h264"
        """
        pieces = filepath.split("/")
        basedir = pieces[1]
        filename = pieces[2]
        pieces = filename.split(".")
        file_time = pieces[0]
        tempfilename = file_time + ".mp4"
        event_time = int(file_time)
        tempfilepath = "/"+ basedir + "/"+ tempfilename
        args = ["/usr/bin/MP4Box", "-fps", "30", "-add", filepath, tempfilepath]
        try:
            retCode = subprocess.call(args);
        except:
	        print "MP4Box failed:" + retCode
        else:
            if (retCode != 0):
                print "Call to MP4Box failed: ", retCode
            else:
                url = "http://"+self.host+"/snoop/events/upload/"+self.unit_id+"/"+file_time
                print url
                files = {'file': (tempfilename, open(tempfilepath, 'rb'))}
                try:
                    r = requests.post(url, files=files, auth=HTTPBasicAuth('hambtw', 'Snoop123'), timeout=180)
                except:
                    printf "Unexpected request error:", sys.exc_info()[0]
                else:
                    print r.status_code
                    #print r.headers
                    print r.text
                print "Removing: " + tempfilepath
                os.remove(tempfilepath)
        print "Removing: " + filepath
        os.remove(filepath)
       
    def ping(self):
        """ Ping the web server of this unit
        """
        url = "http://"+self.host+"/snoop/unitping/"+self.unit_id
        print url
        try:
            r = requests.get(url, auth=HTTPBasicAuth('hambtw', 'Snoop123'))
        except:
            printf "Unexpected request error:", sys.exc_info()[0]
        else:
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
    host = "snoop-env-hjrmvk5eey.elasticbeanstalk.com"
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
