class LowPassFilter:
    """
    Exponential Moving Average Low-Pass Filter
    Computes: y[n] = alpha * x[n] + (1 - alpha) * y[n-1], 
        where x is the input signal, y is the filtered output signal, alpha is the smoothing factor (0 < alpha <= 1)
    Initializes filter to 0.0

    Calling update() repeatedly on the input samples produces a continuous filtered output.
    """

    def __init__(self, alpha: float):
        self.alpha = alpha
        if self.alpha <= 0 or self.alpha > 1:
            raise ValueError("alpha must satisfy 0 < alpha <= 1")
        self.filter_value = 0.0

    def update(self, new_value: float) -> float:
        self.filter_value = self.alpha * new_value + (1 - self.alpha) * self.filter_value
        return self.filter_value

    def value(self) -> float:
        return self.filter_value