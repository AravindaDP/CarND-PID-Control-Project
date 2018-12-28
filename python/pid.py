import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
import json

sio = socketio.Server()
app = Flask(__name__)

class PID:
    def __init__(self):
        """Constructor"""

        """
        Errors
        """
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        """
        Coefficients
        """
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

    def init(self, Kp, Ki, Kd):
        """Initialize PID."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def update_error(self, cte):
        """Update the PID error variables given cross track error."""
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

    def total_error(self):
        """Calculate the total PID error."""
        return 0-(self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error)

class TwiddlingPID(PID):
    def __init__(self, init_dp, max_cte, n):
        self._dp = init_dp
        self._max_cte = max_cte
        self._n = n
        self._tuning_idx = 0
        self._increment = True
        self._error = 0
        self._i = 0
        self.best_error = 10 * max_cte*max_cte

    def update_error(self, cte):
        super(TwiddlingPID, self).update_error(cte)
        self._i = self._i+1
        if self._i > self._n:
            self._error += cte* cte

    def total_error(self):
        pid_error = super(TwiddlingPID, self).total_error()

        dp_sum = sum(self._dp)
        if(dp_sum < 0.001): #Twiddling finished. Continue;
            return pid_error
        if (abs(self.p_error) > self._max_cte and self._i > 10): #Crashed
            pid_error = None
            self._twiddle()
        elif (self._i > 2*self._n):
            pid_error = None
            self._twiddle()

        return pid_error

    def _twiddle(self):
        err = 0.0
        if (self._i <= self.n) :
            err = self._max_cte*self._max_cte
        else:
            err = self._error*self._n/(self._i*(self._i-self._n))

        if self._increment:
            if err < self.best_error:
                self.best_error = err
                self._dp[self._tuning_idx] *= 1.1
                self._next_coefficient()
                self._update_coefficient(self._dp[self._tuning_idx])
            else:
                self._update_coefficient(-2*self._dp[self._tuning_idx])
                self._increment = False
        else:
            if err < self.best_error:
                self.best_error = err
                self._dp[self._tuning_idx] *= 1.1
            else:
                self._update_coefficient(self._dp[self._tuning_idx])
                self._dp[self._tuning_idx] *= 0.9
            self._next_coefficient()
            self._update_coefficient(self._dp[self._tuning_idx])
            self._increment = True
        self._error = 0.0
        self._i = 0
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0

    def _update_coefficient(self, dp):
        if self._tuning_idx == 0:
            self.Kp += dp
        if self._tuning_idx == 1:
            self.Ki += dp
        if self._tuning_idx == 2:
            self.Kd += dp

    def _next_coefficient(self):
        dp_sum = sum(self._dp)
        if(dp_sum > 0.001):
            first_pass = True
            while first_pass or self._dp[self._tuning_idx] < 0.0001:
                first_pass = False
                self._tuning_idx = (self._tuning_idx + 1) % 3

  
def deg2rad(x):
    return math.pi*x/180.0

def rad2deg(x):
    return x*180.0/math.pi


init_dp = [0, 0, 0]
pid = TwiddlingPID(init_dp,4.0,2000)
# Todo: Initialize the pid variable.
init_Kp = 0.1
init_Kd = 5.0
init_Ki = 0.0001
pid.init(init_Kp, init_Ki, init_Kd)
print("Kp: ", pid.Kp, " Ki: ", pid.Ki, " Kd: ", pid.Kd)
_throttle = 0.0
_max_speed = 0.0

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        j = data

        # j is the data JSON object
        cte = float(j["cte"])
        speed = float(j["speed"])
        angle = float(j["steering_angle"])
        steer_value = 0.0
        throtle_value = 0.0

        global _max_speed
        if(speed > _max_speed):
            _max_speed = speed
        
        """
        Todo:
            * Calcuate steering value here, remember the steering value is
              [-1, 1].

        Note:
            Feel free to play around with the throttle and speed. Maybe use
            another PID controller to control the speed!
        """
        pid.update_error(cte)
        steer_value = pid.total_error()

        if steer_value > 1.0:
            steer_value = 1.0
        if steer_value < -1.0:
            steer_value = -1.0
                
        global _throttle

        throtle_value = 0.4
        if speed > 20 and abs(cte) > 0.4 and abs(steer_value) > 0.15:
            throtle_value = -1.0

        if throtle_value > _throttle:
            _throttle = _throttle + 0.2
        elif throtle_value < _throttle:
            _throttle = _throttle - 0.2

        #if throttle > 1.0:
        #    throttle = 1.0
        #if throttle < -1.0:
        #    throttle = -1.0

        
        # DEBUG
        print("CTE: ", cte, " Steering Value: ", steer_value)
        print("Max Speed:", _max_speed)

        msg = "reset"
        msgJson = {}

        if steer_value is None:
            print("Best Error:", pid.best_error)
            print("Kp: ",pid.Kp," Ki: ",pid.Ki," Kd: ",pid.Kd)
            throttle = 0
        else:
            msg = "steer"
            msgJson["steering_angle"] = steer_value
            msgJson["throttle"] = _throttle

        print(msgJson)      
        sio.emit(msg, data=msgJson, skip_sid=True)
    else:
        # Note: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
    print("Connected!!!", sid)

if __name__ == '__main__':
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

