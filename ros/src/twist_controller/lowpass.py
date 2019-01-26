
class LowPassFilter(object):
    def __init__(self, alpha = 0.2):
        self.a = alpha
        self.b = 1 - alpha

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
