def sysCall_init():
    global joint, ui, label_ids
    sim = require('sim')
    global simUI
    simUI = require('simUI')
    
    joint = [sim.getObject('../joint', {'index': i}) for i in range(6)]
    
    limits_deg = [
        (-180, 180),   # J1
        (-180, 0),     # J2
        (-180, 180),   # J3
        (-180, 180),   # J4
        (-180, 180),   # J5
        (-180, 180)    # J6
    ]
    init_vals = [0, -90, 0, 0, 0, 0]
    
    xml = '<ui title="UR5 Joint Sliders" closeable="true" resizable="false" layout="vbox">\n'
    
    joint_names = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    label_ids = []
    
    for i in range(6):
        name = joint_names[i]
        min_val, max_val = limits_deg[i]
        init = init_vals[i]
        
        xml += '    <group layout="vbox" flat="true">\n'
        xml += f'        <label text="Joint {i+1} ({name})" />\n'
        xml += f'        <hslider id="{i+1}" minimum="{min_val}" maximum="{max_val}" value="{init}" />\n'
        xml += f'        <label id="{100+i}" text="{init:.1f}°" style="color: #0066cc; font-weight: bold;" />\n'
        xml += '    </group>\n'
        
        label_ids.append(100 + i)
    
    xml += '</ui>'
    ui = simUI.create(xml)
    
    robot_base = sim.getObject('/UR5')
    all_objects = sim.getObjectsInTree(robot_base, sim.object_shape_type, 0)
    
    for obj in all_objects:
        sim.setShapeColor(obj, None, sim.colorcomponent_transparency, [0.9])
    
    axis_length = 0.1
    axis_radius = 0.003  
    
    for idx, j in enumerate(joint):

        x_axis = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [axis_radius*2, axis_radius*2, axis_length])
        sim.setShapeColor(x_axis, None, sim.colorcomponent_ambient_diffuse, [1, 0, 0])
        sim.setObjectParent(x_axis, j, True)
        sim.setObjectPosition(x_axis, j, [axis_length/2, 0, 0])
        sim.setObjectOrientation(x_axis, j, [0, 1.5708, 0]) 
        sim.setObjectAlias(x_axis, f"Frame_J{idx+1}_X")

        y_axis = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [axis_radius*2, axis_radius*2, axis_length])
        sim.setShapeColor(y_axis, None, sim.colorcomponent_ambient_diffuse, [0, 1, 0])
        sim.setObjectParent(y_axis, j, True)
        sim.setObjectPosition(y_axis, j, [0, axis_length/2, 0])
        sim.setObjectOrientation(y_axis, j, [-1.5708, 0, 0]) 
        sim.setObjectAlias(y_axis, f"Frame_J{idx+1}_Y")

        z_axis = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [axis_radius*2, axis_radius*2, axis_length])
        sim.setShapeColor(z_axis, None, sim.colorcomponent_ambient_diffuse, [0, 0, 1])
        sim.setObjectParent(z_axis, j, True)
        sim.setObjectPosition(z_axis, j, [0, 0, axis_length/2])
        sim.setObjectOrientation(z_axis, j, [0, 0, 0])
        sim.setObjectAlias(z_axis, f"Frame_J{idx+1}_Z")

def sysCall_actuation():
    for i in range(6):
        deg = simUI.getSliderValue(ui, i + 1)
        rad = deg * 3.141592653589793 / 180
        sim.setJointTargetPosition(joint[i], rad)
        simUI.setLabelText(ui, label_ids[i], f"{deg:.1f}°")

def sysCall_cleanup():
    simUI.destroy(ui)