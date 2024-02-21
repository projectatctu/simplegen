import lxml.etree as ET

from typing import Dict, Union


def xml_set_value(
    elem: ET.SubElement,
    path: str,
    text: Union[Dict[str, str], str],
    values: Dict[str, str] = {},
) -> None:
    assert len(path) != 0
    subelements = path.split("/")

    if subelements[0] in values:
        el = ET.SubElement(elem, subelements[0], name=values[subelements[0]])
    else:
        el = ET.SubElement(elem, subelements[0])

    for i in range(1, len(subelements)):
        if subelements[i] in values:
            el = ET.SubElement(el, subelements[i], name=values[subelements[0]])
        else:
            el = ET.SubElement(el, subelements[i])

    if isinstance(text, str):
        el.text = text

    if isinstance(text, dict):
        for key, t in text.items():
            k = ET.SubElement(el, key)
            k.text = t
