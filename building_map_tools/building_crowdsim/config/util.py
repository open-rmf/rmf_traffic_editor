import xml.etree.ElementTree as ET
import os

# use indent for '\t' and newline for '\n'
def prettyXml(element, indent, newline, level = 0) :
    # check whether the element has a child element
    if element :
        if element.text == None or element.text.isspace():
            element.text = newline + indent * (level + 1)
        else :
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)
    
    temp = list(element)
    for subelement in temp :
        # not the last one
        if temp.index(subelement) < (len(temp) - 1) :
            subelement.tail = newline + indent * (level + 1)
        else:
            # finishing the current element, the indent becomes the previous element
            subelement.tail = newline + indent * level
        
        prettyXml(subelement, indent, newline, level = level + 1)


def writeXmlFile(root_element, output_dir = '', file_name = '') :
    if len(output_dir) == 0:
        print("Warning: 'output_dir' is not specified. Write file to ", os.getcwd())
        output_dir = os.getcwd()
    else :
        if output_dir[0] == '/':
            print("Generate", file_name, "to ", output_dir)
        else: 
            print("Generate", file_name, "to ", os.getcwd() + '/' + output_dir)

    writeXMLtoCompleteFilePath(root_element, output_dir + '/' + file_name)


def writeXMLtoCompleteFilePath(root_element, file_name = '') :
    if not ET.iselement(root_element) :
        raise ValueError("Failed to write xml file. Invalid element provided!")

    prettyXml(root_element, '\t', '\n')

    # ET.dump(root_element)
    tree = ET.ElementTree(root_element)
    tree.write(file_name, encoding='utf-8')


def templateYamlFile(level_name, tree, output_file, write_mode) :
    filehandle = open(output_file, write_mode)
    newline = '\n'
    indent = ' '
    
    indent_level = 0
    templatePrettyYaml(filehandle, indent_level, level_name + ":")
    indent_level = indent_level + 1

    for key in tree :
        # print key
        templatePrettyYaml(filehandle, indent_level, key + ":")
        items = tree[key]
        for item in items:
            if key == "goals" :
                content_text = "- [" + str(item.x) + ", " + str(item.y) + ", " + str(item.z) + ", " + str(item.name) + "]"
            elif key == "goal_area" :
                content_text = "- " + item
            else:
                content_text = "- {"
                for k in item :
                    content_text += k + ": " + str(item[k]) + ", "
                
                # delete the last ", "
                content_text = content_text[0:-2] 
                content_text += "}"

            templatePrettyYaml(filehandle, indent_level+1, content_text)

    filehandle.close()

def templatePrettyYaml(filehandle, indent_level, content_text):
    newline = '\n'
    indent = ' '

    if filehandle.closed :
        raise OSError("file is already closed.")
    
    filehandle.write( newline + indent * indent_level + content_text)

    


