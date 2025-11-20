class LowPassFilter:
    """
    Exponential Moving Average Low-Pass Filter
        computes: y[n] = alpha*x[n] + (1 - alpha)*y[n-1]

    Calling update() repeatedly produces a continuous filtered output.
    """

    def __init__(alpha: float):
        self.alpha = alpha
        if self.alpha <= 0 or self.alpha > 1:
            raise ValueError("alpha must satisfy 0 < alpha <= 1")
        self.prev = 0.0

    def update(self, new_value: float) -> float:
        self.prev = self.alpha * new_value + (1 - self.alpha) * self.prev
        return self.prev

    def value(self) -> float:
        return self.prev