class Store:
    def __init__(self):
        super().__init__()
        self.data = {}
    def update(self, t, **kwargs):
        for key, value in kwargs.items():
            if key in self.data:
                self.data[key].append((t,value))
            else:
                self.data[key] = [(t, value)]

    def __getitem__(self, key):
        return self.data[key]

    def keys(self):
        return list(self.data.keys())
