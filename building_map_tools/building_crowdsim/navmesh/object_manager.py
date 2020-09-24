class Manager:

    def __init__(self):
        self.data = []

    def add_obj(self, obj):
        assert(hasattr(obj, 'id')), "Adding object without id attribute!"
        obj.id = len(self.data)
        self.data.append(obj)
