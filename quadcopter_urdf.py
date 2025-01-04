import tempfile

def box(x_size, y_size, z_size):
    return f'<geometry><box size="{x_size} {y_size} {z_size}" /></geometry>'


def cylinder(radius, length):
    return f'<geometry><cylinder radius="{radius}" length="{length}" /></geometry>'


def origin(x, y, z):
    return f'<origin xyz="{x} {y} {z}" rpy="0 0 0" />'


def visual_collision(item: str, x: float, y: float, z: float, color: str):
    return f'''
    <visual>
      {origin(x, y, z)}
      {item}
      <material name="{color}"/>
    </visual>
    <collision>
      {origin(x, y, z)}
      {item}
    </collision>
    '''

def inertial(mass: float, x: float, y: float, z: float):
    return f'''
    <inertial>
      <mass value="{mass}" />
      {origin(x, y, z)}
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0" />
    </inertial>
    '''

def fixed_joint(parent: str, child: str):
    name = f'{parent}_{child}'
    return f'<joint name="{name}" type="fixed"><parent link="{parent}" /><child link="{child}" /></joint>'



def quadcopter_urdf(width, length, height, mass, x_center_of_mass = 0.0, y_center_of_mass = 0.0, z_center_of_mass = 0.0, propeller_diameter = 0.12, propeller_height = 0.01):

    half_width = width / 2
    half_length = length / 2

    propeller = cylinder(propeller_diameter, propeller_height)

    content = f'''
    <robot name="drone">
        <link name="base">
            {visual_collision(box(length, width, height), 0, 0, 0, "gray")}
            {inertial(mass, x_center_of_mass, y_center_of_mass, z_center_of_mass)}
        </link>
        <link name="fl">
            {visual_collision(propeller, half_length, half_width, 0, "red")}
            {inertial(0, 0, 0, 0)}
        </link>
        <link name="fr">
            {visual_collision(propeller, half_length, -half_width, 0, "blue")}
            {inertial(0, 0, 0, 0)}
        </link>
        <link name="bl">
            {visual_collision(propeller, -half_length, half_width, 0, "gray")}
            {inertial(0, 0, 0, 0)}
        </link>
        <link name="br">
            {visual_collision(propeller, -half_length, -half_width, 0, "gray")}
            {inertial(0, 0, 0, 0)}
        </link>
        {fixed_joint("base", "fl")}
        {fixed_joint("base", "fr")}
        {fixed_joint("base", "bl")}
        {fixed_joint("base", "br")}

        <material name="gray"><color rgba="0.8 0.8 0.8 1"/></material>
        <material name="red"><color rgba="1 0 0 1"/></material>
        <material name="blue"><color rgba="0 0 1 1"/></material>
    </robot>
    '''
    
    with tempfile.NamedTemporaryFile(delete=False, suffix='.urdf') as f:
        f.write(bytes(content, 'utf-8'))
        return f.name
