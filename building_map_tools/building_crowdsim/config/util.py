import xml.etree.ElementTree as ET
import os


# use indent for '\t' and newline for '\n'
def pretty_xml(element, indent, newline, level=0):
    # check whether the element has a child element
    if element:
        if element.text is None or element.text.isspace():
            element.text = newline + indent * (level + 1)
        else:
            element.text =\
                newline +\
                indent * (level + 1) +\
                element.text.strip() +\
                newline +\
                indent * (level + 1)
    temp = list(element)
    for subelement in temp:
        # not the last one
        if temp.index(subelement) < (len(temp) - 1):
            subelement.tail = newline + indent * (level + 1)
        else:
            # finishing the current element
            # the indent becomes the previous element
            subelement.tail = newline + indent * level
        pretty_xml(subelement, indent, newline, level=level + 1)


def write_xml_file(root_element, output_dir='', file_name=''):
    if len(output_dir) == 0:
        print("Warning: 'output_dir' is not specified. Write file to ",
              os.getcwd())
        output_dir = os.getcwd()
    else:
        if output_dir[0] == '/':
            print("Generate", file_name,
                  "to ", output_dir)
        else:
            print("Generate", file_name,
                  "to ", os.getcwd() + '/' + output_dir)
    write_xml_to_complete_file_path(
        root_element, output_dir + '/' + file_name)


def write_xml_to_complete_file_path(root_element, file_name=''):
    if not ET.iselement(root_element):
        raise ValueError("Failed to write xml file. Invalid element provided!")
    pretty_xml(root_element, '\t', '\n')
    tree = ET.ElementTree(root_element)
    tree.write(file_name, encoding='utf-8')
