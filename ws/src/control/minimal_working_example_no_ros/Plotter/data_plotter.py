import matplotlib.pyplot as plt
class DataPlotter:
    def __init__(self, num_subplots=1):
        self.num_subplots = num_subplots
        self.fig, self.axes = plt.subplots(num_subplots, 1, figsize=(8, 4 * num_subplots))
        if num_subplots == 1:
            self.axes = [self.axes]
    def plot_series(self, subplot_index, x, y, label=None):
        self.axes[subplot_index].plot(x, y, label=label)
        self.axes[subplot_index].legend()
    def show_plots(self):
        plt.show()

