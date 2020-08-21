import xml.etree.ElementTree as ET

class LeafElement:
    
    def __init__(self, name) :
        self._name = name
        self._attrib = {}
        self._text = None
        
    def addAttribute(self, key, value) :
        self._attrib[str(key)] = str(value)

    def setAttributes(self, attributes) :
        for key in attributes:
            self.addAttribute(key, attributes[key])

    def setText(self, text):
        self._text = str(text)

    def getText(self) :
        return self._text

    def outputXmlElement(self):
        root = ET.Element(self._name, self._attrib)
        if self._text :
            root.text = self._text
        return root


class Element (LeafElement):
    def __init__(self, name):
        LeafElement.__init__(self, name)
        self._subElements = []

    def addSubElement(self, element) :
        if not hasattr(element, "outputXmlElement"):
            raise ValueError(element + " does not have method 'outputXmlElement' ")
        
        self._subElements.append(element)
    
    def outputXmlElement(self):
        root = ET.Element(self._name, self._attrib)

        for element in self._subElements :
            root.append(element.outputXmlElement())
        
        return root
