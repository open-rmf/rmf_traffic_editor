import xml.etree.ElementTree as ET


class LeafElement:
    def __init__(self, name):
        self.name = name
        self.attributes = {}
        self.text = None

    def is_valid(self):
        return True

    def output_xml_element(self):
        if not self.is_valid():
            raise ValueError("Element " +
                             self.name +
                             " valid check failed")
        for key in self.attributes:
            self.attributes[key] = str(self.attributes[key])
        root = ET.Element(self.name, self.attributes)
        if self.text:
            root.text = self.text
        return root


class Element (LeafElement):
    def __init__(self, name):
        LeafElement.__init__(self, name)
        self.sub_elements = []

    def is_valid(self):
        for sub_elem in self.sub_elements:
            if not sub_elem.is_valid():
                return False
        return True

    def output_xml_element(self):
        if not self.is_valid():
            raise ValueError("Element " +
                             self.name +
                             " valid check failed")
        for key in self.attributes:
            self.attributes[key] = str(self.attributes[key])
        root = ET.Element(self.name, self.attributes)
        for element in self.sub_elements:
            root.append(element.output_xml_element())
        return root
