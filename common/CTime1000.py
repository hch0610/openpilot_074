
import time


class CTime1000:
    def __init__(self, txt_msg="time", end_time = 0 ):
        self.name = txt_msg
        self.start_time = time.time()
        self.end_time = self.start_time + end_time
        self.old_txt = ""
        self.debug_Timer = 0

    def __del__(self):
        print( "{} - class delete".format( self.name ))

    def get_time(self):
        cur_time = time.time()
        delta_time = self.end_time - cur_time

        return delta_time


    def startTime(self, end_time ):
        self.start_time = time.time()
        self.end_time = self.start_time + end_time

    def endTime(self, end_time = -1):
        ret_code = False
        if  end_time > 0:
            self.end_time = self.start_time + end_time

        cur_time = time.time()
        delta_time = self.end_time - cur_time
        if delta_time < 0.:
              ret_code = True

        return  ret_code




