from abc import ABC, abstractmethod, abstractproperty

class DataSource(ABC):
    signals_dict = {}

    @property
    @abstractmethod
    def name(self):
        pass

    @property
    @abstractmethod
    def signals(self):
        pass

    @property
    @abstractmethod
    def signal_names(self):
        pass

    @abstractmethod
    def update_signals(self, current_signals):
        pass
    def update_signals_dict(self):
        self.signals_dict[self.name] = self.signals


