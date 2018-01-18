
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val

# Credits: https://www.swharden.com/wp/2008-11-17-linear-data-smoothing-in-python/

class SmoothingFilter(object):
	
	def __init__(self, window_weight):
		
		self.last_val = 0
		self.window_weight = window_weight
		
	
	def get_smoothed_value(self, value):
		
		self.last_val = (self.window_weight * self.last_val) + ((1.0 - self.window_weight) * value)
		return self.last_val
		
