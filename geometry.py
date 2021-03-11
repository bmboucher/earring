from os import remove
import xml.etree.ElementTree as ET
from math import cos, sin, pi, sqrt, atan2

tree = ET.parse('hoop_v2.brd')
root = tree.getroot()

OUTER_RADIUS = 28
CENTER = OUTER_RADIUS
PAD_RADIUS = 24.1
INNER_RADIUS = 19
TOP_LAYER = '1'
DIM_LAYER = '20'
TSTOP_LAYER = '29'
DIM_WIDTH = 0.1

def place_element(el: ET.Element, r: float, theta: float, rot: float):
    el.set('x', str(CENTER + r * cos(2 * pi * theta)))
    el.set('y', str(CENTER + r * sin(2 * pi * theta)))
    el.set('rot', f'R{int(round((theta + rot) * 360))}')

def clean_layer(root_name: str, layer: str) -> ET.Element:
    el = root.find(root_name)
    if not el:
        return root
    layer_nodes = [n for n in el if n.attrib.get('layer', None) == layer]
    for n in layer_nodes:
        el.remove(n)
    return el

N_LEDS = 60

LED1_ANGLE = float(20) / N_LEDS
LED_RADIUS = 21.5

def set_leds_and_resistors():
    for el in root.iter('element'):
        name = el.attrib['name']
        if name.startswith('D'):
            n = int(name[1:])
            print(f'Placing LED {name}')
            place_element(el, LED_RADIUS, float(n - 1) / N_LEDS + LED1_ANGLE, 0.25)
        elif name.startswith('R'):
            continue
            n = int(name[1:])
            print(f'Placing resistor {name}')
            place_element(el, LED_RADIUS, float(2 * n - 1) / N_LEDS, LED1_ANGLE)
            for s in el.findall('attribute'):
                if s.attrib['name'] in ['NAME', 'VALUE']:
                    el.remove(s)

def create_outline():
    plain_node = clean_layer('drawing/board/plain', DIM_LAYER)

    plain_node.append(ET.Element('circle', x=str(CENTER), y=str(CENTER),
        radius=str(INNER_RADIUS), width=str(DIM_WIDTH), layer=DIM_LAYER))
    outer_points = [(str(CENTER + OUTER_RADIUS * cos(n * pi / 6)), str(CENTER + OUTER_RADIUS * sin(n * pi / 6))) for n in range(12)]
    for i in range(12):
        x1, y1 = outer_points[i]
        x2, y2 = outer_points[i + 1] if i < 11 else outer_points[0]
        plain_node.append(ET.Element('wire', x1=x1, y1=y1, x2=x2, y2=y2, width=str(DIM_WIDTH), layer=DIM_LAYER, curve='-90'))

PAD_ARC_STEPS=50
def create_pads():
    plain_node = clean_layer('drawing/board/plain', TSTOP_LAYER)
    for i in range(12):
        base_angle = pi * i / 6
        min_angle = base_angle - pi / 12

        polygon_node = ET.Element('polygon', width='0.01', layer=TSTOP_LAYER)
        for j in range(PAD_ARC_STEPS + 1):
            theta = min_angle + (pi * j)/(PAD_ARC_STEPS * 6)
            x = CENTER + PAD_RADIUS * cos(theta)
            y = CENTER + PAD_RADIUS * sin(theta)
            polygon_node.append(ET.Element('vertex', x=str(x), y=str(y)))
        polygon_node.append(ET.Element('vertex',
            x=str(CENTER+OUTER_RADIUS * cos(base_angle)),
            y=str(CENTER+OUTER_RADIUS * sin(base_angle))))
        plain_node.append(polygon_node)

def create_signal_pads():
    signals_node = root.find('drawing/board/signals')
    signals = [s for s in signals_node.iter('signal')
               if s.attrib.get('name').startswith('PAD')]
    for s in signals:
        signals_node.remove(s)

    for i in range(12):
        base_angle = pi * i / 6
        min_angle = base_angle - pi / 12

        polygon_node = ET.Element('polygon', width='0.01', layer=TOP_LAYER)
        for j in range(2,PAD_ARC_STEPS - 1):
            theta = min_angle + (pi * j)/(PAD_ARC_STEPS * 6)
            x = CENTER + PAD_RADIUS * cos(theta)
            y = CENTER + PAD_RADIUS * sin(theta)
            polygon_node.append(ET.Element('vertex', x=str(x), y=str(y)))
        polygon_node.append(ET.Element('vertex',
            x=str(CENTER+OUTER_RADIUS * cos(base_angle)),
            y=str(CENTER+OUTER_RADIUS * sin(base_angle))))
        signal_node = ET.Element('signal', name=f'PAD{i}')
        signal_node.append(polygon_node)
        signals_node.append(signal_node)

def fix_airwires():
    signals_node = root.find('drawing/board/signals')
    for signal_node in signals_node.findall('signal'):
        print(signal_node.attrib['name'])
        if not signal_node.attrib['name'].startswith('N$'):
            continue
        if all(c.attrib['element'].startswith('D') for c in 
                    signal_node.findall('contactref')):
            print(f"{signal_node.attrib['name']} - PASS")
            for wire_node in signal_node.iter('wire'):
                if wire_node.attrib['layer'] != '1':
                    print(f'Fixing airwire {signal_node.attrib["name"]}')
                    wire_node.attrib['width'] = '0.1524'
                    wire_node.attrib['layer'] = '1'
                    wire_node.attrib.pop('extent')

LED_PAD_OFFSET = 0.55

VCC_PAD_RADIUS = sqrt((LED_RADIUS + LED_PAD_OFFSET) ** 2 + LED_PAD_OFFSET ** 2)
VCC_PAD_ANGLE = (atan2(LED_PAD_OFFSET, LED_RADIUS + LED_PAD_OFFSET)) / (2 * pi)
VCC_BUS_RADIUS = VCC_PAD_RADIUS + 1.2

def create_radial_wire(r1: float, theta1: float, r2: float, theta2: float,
                       layer: str, width: float) -> ET.Element:
    return ET.Element('wire', 
        x1=str(CENTER + r1*cos(theta1 * 2 * pi)),
        y1=str(CENTER + r1*sin(theta1 * 2 * pi)), 
        x2=str(CENTER + r2*cos(theta2 * 2 * pi)), 
        y2=str(CENTER + r2*sin(theta2 * 2 * pi)),
        layer=layer, width=str(width))

def create_arc_wire(r: float, theta1: float, theta2: float, layer: str, width: float) -> ET.Element:
    w = create_radial_wire(r, theta1, r, theta2, layer, width)
    w.attrib['curve'] = str((theta2 - theta1) * 360)
    return w

def add_led_pwr():
    signals_node = root.find('drawing/board/signals')
    vcc_node = next(n for n in signals_node if n.attrib['name'] == 'VCC')
    vcc_top_wires = [w for w in vcc_node.iter('wire') 
                     if w.attrib['layer'] == TOP_LAYER]
    print(f'Found {len(vcc_top_wires)} wires')
    for w in vcc_top_wires:
        vcc_node.remove(w)
    for n in range(N_LEDS):
        theta = LED1_ANGLE + (float(n) / N_LEDS) + VCC_PAD_ANGLE
        vcc_node.append(create_radial_wire(
            VCC_PAD_RADIUS, theta, VCC_BUS_RADIUS, theta, TOP_LAYER, 0.3))
        vcc_node.append(create_arc_wire(
            VCC_BUS_RADIUS, theta, theta + 1.0 / N_LEDS,
            TOP_LAYER, 0.9
        ))

GND_PAD_RADIUS = sqrt((LED_RADIUS - LED_PAD_OFFSET) ** 2 + LED_PAD_OFFSET ** 2)
GND_PAD_ANGLE = (atan2(-LED_PAD_OFFSET, LED_RADIUS - LED_PAD_OFFSET)) / (2 * pi)
GND_BUS_RADIUS = GND_PAD_RADIUS - 1.2

def add_led_gnd():
    signals_node = root.find('drawing/board/signals')
    gnd_node = next(n for n in signals_node if n.attrib['name'] == 'GND')
    gnd_top_wires = [w for w in gnd_node.iter('wire') 
                     if w.attrib['layer'] == TOP_LAYER]
    print(f'Found {len(gnd_top_wires)} wires')
    for w in gnd_top_wires:
        gnd_node.remove(w)
    for n in range(N_LEDS):
        theta = LED1_ANGLE + (float(n) / N_LEDS) + GND_PAD_ANGLE
        gnd_node.append(create_radial_wire(
            GND_PAD_RADIUS, theta, GND_BUS_RADIUS, theta, TOP_LAYER, 0.3))
        gnd_node.append(create_arc_wire(
            GND_BUS_RADIUS, theta, theta + 1.0 / N_LEDS,
            TOP_LAYER, 0.9
        ))


def daisychain_leds():
    signals_node = root.find('drawing/board/signals')

    def is_data_pin(c: ET.Element):
        return c.attrib['element'].startswith('D') and c.attrib['pad'] in ['1','3']

    for signal_node in signals_node.iter('signal'):
        contacts = list(signal_node.iter('contactref'))
        if len(contacts) == 2 and all(map(is_data_pin, contacts)):
            theta = float(int(next(c.attrib['element'] for c in contacts if c.attrib['pad'] == '1')[1:]) - 1) / N_LEDS + LED1_ANGLE
            wires = list(signal_node.iter('wire'))
            for w in wires:
                signal_node.remove(w)
            signal_node.append(create_radial_wire(
                GND_PAD_RADIUS, theta - GND_PAD_ANGLE,
                VCC_PAD_RADIUS, theta - VCC_PAD_ANGLE + (1.0 / N_LEDS),
                TOP_LAYER, 0.1524
            ))

set_leds_and_resistors()
daisychain_leds()
#fix_airwires()
add_led_pwr()
add_led_gnd()
tree.write('hoop_v2.brd')

print('DONE')