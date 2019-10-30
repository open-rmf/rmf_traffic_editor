def indent_etree(elem, level=0):
    '''Indent an etree so it looks nice for humans to read.

    Inspired by https://stackoverflow.com/a/33956544
    Not sure if this is the best way, but it seems to work fine.
    '''
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent_etree(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
