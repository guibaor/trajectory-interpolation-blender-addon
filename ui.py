##############################################################################
# Animation Project 1
#
# Guillem Barceló Orts
# Ana Gloria Galvez Mellado
# Darío Rodríguez Hernández
# Bekzat Mukhamedali
#
##############################################################################
import bpy, os, sys, importlib


script_file = os.path.realpath(__file__)
dir = os.path.dirname(script_file)
if not dir in sys.path:
     sys.path.append(dir)

import interpolations_and_ops as Interpol_functions

importlib.reload(Interpol_functions)

##############################################################################
# Operators
##############################################################################
class Calculate(bpy.types.Operator):
    """
    Class Operator that execute the interpolation between keyframes
    taking account all the custom property parameters.
    Attributes
    ----------
    bl_idname : str
        Name that reference this operatir
    bl_label : str
        Name that appears in the Operator's button at the interface

    Methods
    ----------
    poll(cls,context)
        Check a precondition group to able or disable the oparator's button at the interface

    invoke(self,context)
        Run the interpolation between the keyframes of an object action.
    """

    bl_idname = "object.calculate"
    bl_label = "Calculate interpolation"

    @classmethod
    def poll(cls, context):
        active = True;
        if (context.active_object is None):
            active = False
        # If there is no actions.
        elif (not bpy.context.object.animation_data):
            active = False
        active = False
        for action in bpy.data.actions:
            if (action.name == context.active_object.action_name):
                active = True
        # If there is less than 2 location keyframes
        if (active):
            if (bpy.data.actions[bpy.data.objects[context.active_object.name].action_name].fcurves.find('location',index=0) is None):
                active = False
            elif (len(bpy.data.actions[bpy.data.objects[context.active_object.name].action_name].fcurves.find('location',index=0).keyframe_points) <= 1):
                active = False

        # Checkings when oscilation is active
        if (bpy.data.objects[context.active_object.name].oscilation):
            # If the main axis is the same than the lateral axis -> False
            if(bpy.data.objects[context.active_object.name].main_axis ==
            bpy.data.objects[context.active_object.name].inclination_axis):
                active = False
            # If the main axis is the same than Lateral axis but negative -> False
            elif(bpy.data.objects[context.active_object.name].main_axis ==
            ( ("-") + bpy.data.objects[context.active_object.name].inclination_axis)): # Hemos puesto paréntesis al "-" para que sea un emoji ("-")
                active = False
            # If the main axis is the same than Lateral axis but positive -> False
            elif(("-" + bpy.data.objects[context.active_object.name].main_axis) ==
            bpy.data.objects[context.active_object.name].inclination_axis):
                active = False
        return active

    def execute(self, context):
        Interpol_functions.calculate()
        return {'FINISHED'}

##############################################################################
# Panel
##############################################################################
class Interpolation(bpy.types.Panel):
    """
    Creates a Panel in the scene context of the properties editor

    Attributes
    ----------
    bl_idname : str
        Name that reference this operatir
    bl_label : str
        Name that appears in the Operator's button at the interface
    bl_space_type : str
        Blender space where the panel will be inserted
    bl_region_type : str
        Displaying mode of the panel
    bl_context: str
        Work ambit of blender

    Methods
    ----------
    draw(self,context)
        Draws the panel with all the interactive elements to change the custom properties
    """

    bl_label = "Trayectory Effect"
    bl_idname = "SCENE_PT_layout"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw(self, context):
        """
        Draws the panel with all the interactive elements to change the custom properties
        Parameters
        ----------
        self : pointer
            Pointer that points to itself
        context : blender_context
            Work ambit of blender
        """
        layout = self.layout

        obj = context.object

        # Master action
        row = layout.row()
        row.label(icon="ACTION")
        row.prop(obj, "action_name")

        row = layout.row()
        row.separator()

        # Interpolation method
        row = layout.row()
        row.prop(obj, "interpolation")

        # Oscilation
        row = layout.row()
        row.prop(obj, "oscilation")

        # Amplitude
        if (obj.oscilation == True):
            row = layout.row()
            row.prop(obj, "amplitude")

        # Frequency
        row = layout.row()
        row.prop(obj, "frequency")

        # Tension
        if (obj.interpolation == "CAT"):
            row = layout.row()
            row.prop(obj, "tau")

        row = layout.row()
        row.separator()

        # Velocity settings
        row = layout.row()
        row.prop(obj, "velocity_settings")

        # Constant vel. and reparam vel.
        if (obj.velocity_settings == True):
            row = layout.row()
            row.prop(obj, "constant")
            if (obj.constant == True):
                row.prop(obj, "velocity_value")
            row = layout.row()
            row.prop(obj, "apply_reparam")
            if (obj.apply_reparam == True):
                row.prop(obj, "distance")

        row = layout.row()
        row.prop(obj, "orientate")

        if (obj.orientate== True):
            row = layout.row()
            row.prop(obj, "main_axis")
            row.prop(obj, "inclination_axis")
            row = layout.row()
            row.prop(obj, "inclination")
            if (obj.inclination== True):
                row = layout.row()
                row.prop(obj, "inclination_angle")
        row.separator()

        # Calculate button
        row = layout.row()
        row.operator("object.calculate", text="Calculate Trajectory")


def register():
    """
    Register all the previous defined classes and the custom properties on Blender

    """
    bpy.utils.register_class(Interpolation)
    bpy.utils.register_class(Calculate)

    # Interpolation methods
    bpy.types.Object.interpolation = bpy.props.EnumProperty(items=[
    ('LIN', "Lineal", "Lineal interpolation", 1),
    ('HER', "Hermite", "Hermite interpolation", 2),
    ('CAT', "Catmull-Rom", "Catmull-Rom interpolation", 3)],
    name="Interpolation", description="Interpolation method to use", default='LIN')

    # Object oscilation amplitude
    bpy.types.Object.amplitude = bpy.props.FloatProperty(
    name="Amplitude",
    description = "How exagerate you want the oscilation to be",
    default=0.5,
    min = 0.0,
    max = 10.0)

    # Object oscilation frequency
    bpy.types.Object.frequency = bpy.props.IntProperty(
    name="Frequency",
    description = "Amount of intermediate keyframes inserted in between the original ones",
    default=45,
    min = 1,
    max = 50)

    # Object Catmun-roll tau
    bpy.types.Object.tau = bpy.props.FloatProperty(
    name="Tau",
    description = "Modifies the velocity in all paths",
    default=0.2,
    min = 0.0,
    max = 0.5)

    # Random oscilation
    bpy.types.Object.oscilation = bpy.props.BoolProperty(
    name="Oscilation",
    description = "A random oscilation added to the trajectory",
    default = False)

    # Master trajectory
    bpy.types.Object.action_name = bpy.props.StringProperty(
    name="Master action",
    description = "Name of the master trajectory",
    default="CubeAction")

    # Settings
    bpy.types.Object.velocity_settings = bpy.props.BoolProperty(
    name="Velocity settings",
    description = "Enable velocity settings",
    default=False)

    # Constant velocity
    bpy.types.Object.constant = bpy.props.BoolProperty(
    name="Constant velocity",
    description = "Select constant velocity or variable velocity.",
    default=False)

    # Constant velocity value
    bpy.types.Object.velocity_value = bpy.props.FloatProperty(
    name="Value",
    description = "Select constant velocity value",
    default=1.0,
    min = 1.0)

    # Control velocity
    bpy.types.Object.apply_reparam = bpy.props.BoolProperty(
    name="Control velocity",
    description = "Select velocity controlled by distance",
    default=False)

    # Traveled distace in a frame
    bpy.types.Object.distance = bpy.props.FloatProperty(
    name="Current distance",
    description = "Select traveled distance on current frame.",
    default=0.001,
    min = 0.001)

    # Orientate object
    bpy.types.Object.orientate = bpy.props.BoolProperty(
    name="Orientation",
    description = "Select an orientation for the object",
    default=False)

    # Axis to orientate
    bpy.types.Object.main_axis = bpy.props.EnumProperty(items=[
    ("X", "x", "", 1),
    ("-X", "-x", "", 2),
    ("Y", "y", "", 3),
    ("-Y", "-y", "", 4),
    ("Z", "z", "", 5),
    ("-Z", "-z", "", 6)],
    name="Main Axis", description="Axis to orientate", default='Z')

    # Enable/Disable inclination
    bpy.types.Object.inclination = bpy.props.BoolProperty(
    name="Inclination",
    description = "Enable/Disable the inclination with a wraping angle.",
    default=False)

    # Axis for inclination
    bpy.types.Object.inclination_axis = bpy.props.EnumProperty(items=[
    ("X", "x", "", 1),
    ("-X", "-x", "", 2),
    ("Y", "y", "", 3),
    ("-Y", "-y", "", 4),
    ("Z", "z", "", 5),
    ("-Z", "-z", "", 6)],
    name="Lateral axis", description="Axis for inclination", default='X')

    # Value of the wraping angle
    bpy.types.Object.inclination_angle = bpy.props.FloatProperty(
    name="Wrapping angle",
    description = "Value of the wrapping angle in degrees.",
    default= 0.0,
    min= -360.0,
    max= 360.0)

def unregister():
    """
    Unregister all the previous defined classes and the custom properties on Blender

    """
    bpy.utils.unregister_class(Interpolation)
    bpy.utils.unregister_class(Calculate)
    del bpy.types.Object.interpolation
    del bpy.types.Object.amplitude
    del bpy.types.Object.frequency
    del bpy.types.Object.tau
    del bpy.types.Object.oscilation
    del bpy.types.Object.action_name
    del bpy.types.Object.velocity_settings
    del bpy.types.Object.velocity_value
    del bpy.types.Object.constant
    del bpy.types.Object.apply_reparam
    del bpy.types.Object.distance
    del bpy.types.Object.orientate
    del bpy.types.Object.inclination
    del bpy.types.Object.inclination_angle
    del bpy.types.Object.main_axis
    del bpy.types.Object.inclination_axis
