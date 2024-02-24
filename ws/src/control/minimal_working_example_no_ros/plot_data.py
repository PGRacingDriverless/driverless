import pickle
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from minimal_working_example_no_ros.Plotter.data_plotter import DataPlotter

def tuples_to_timeseries(samples):
    timestamps, values = zip(*samples)
    return timestamps, values

def plot_signals(plant_data):
    root = tk.Tk()
    root.title("Plant data plotter")

    notebook = ttk.Notebook(root)
    notebook.pack(fill='both', expand=True)

    for object_name, signals in plant_data.items():
        frame = ttk.Frame(notebook)
        notebook.add(frame, text=object_name)

        fig, axes = plt.subplots(len(signals), 1, figsize=(8, 4 * len(signals)), sharex=True)
        if len(signals) == 1:
            axes = [axes]  # Ensure it's a list even if there's only one subplot

        for i, (signal_name, samples) in enumerate(signals.items()):
            timestamps, values = zip(*samples)
            axes[i].plot(timestamps, values, label=signal_name)
            axes[i].set_ylabel(signal_name)
            axes[i].legend()

        axes[-1].set_xlabel('Time')
        plt.suptitle(f"{object_name} Signals")

        # Embed Matplotlib figure in Tkinter
        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    root.mainloop()


if __name__ == "__main__":
    data_plotter = DataPlotter(num_subplots=1)
    with open("./logs/longitudinal_pid_test.pkl", "rb") as pickle_file:
        loaded_data = pickle.load(pickle_file)
    # plot data
    plot_signals(loaded_data)

