from abc import ABC, abstractmethod

class Output(ABC):
    def __init__(self):
        super().__init__()

    @abstractmethod
    def update(self, **kwargs):
        pass
