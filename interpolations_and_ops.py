##############################################################################
# Animation Project 1
#
# Guillem Barceló Orts
# Ana Gloria Galvez Mellado
# Darío Rodríguez Hernández
# Bekzat Mukhamedali
#
##############################################################################

import bpy
import mathutils
import random
import math
import copy

##############################################################################
# Functions
##############################################################################

def interp_lin(u, x1, x2):
    '''Interpolate two values linearly

    Parameters
    ----------
    u : float
        Relative position
    x1 : float
        First value to interpolate
    x2 : float
        Second value to interpolate

    Returns
    -------
    float
        a float with the interpolated value
    '''
    x = x1 + u * (x2 - x1)
    return x

def inter_hermite(u, x1, x2, v1, v2):
    '''Interpolate two values (two positions & two velocities) using Hermite

    Parameters
    ----------
    u : float
        Relative position
    x1 : float
        First point to interpolate
    x2 : float
        Second point to interpolate
    v1 : float
        First point velocity
    v2 : float
        Second point velocity

    Returns
    -------
    float
        a float with the interpolated value
    '''
    x = (1 - 3 * (u**2) + 2 * (u**3)) * x1 + (u**2) * (3 - 2 * u) * x2 + \
    u * ((u - 1)**2) * v1 + (u**2) * (u - 1) * v2
    return x

def inter_catmull(u, x0, x1, x2, x3, tau):
    '''Interpolate two values using Catmull-Rom

    Parameters
    ----------
    u : float
        Relative position
    x0 : float
        Point before the first point to interpolate
    x1 : float
        First point to interpolate
    x2 : float
        Second point to interpolate
    x3 : float
        Point after the second point to interpolate
    tau : float
        Catmull-rom tension

    Returns
    -------
    float
        a float with the interpolated value
    '''
    v1 = (x2 - x0) * tau
    v2 = (x3 - x1) * tau
    x = inter_hermite(u, x1, x2, v1, v2)
    return x

def interpolate_values(f, f1, f2, x0, x1, x2, x3, v1, v2, tau, method = 'LIN'):
    '''Interpolate two values using the specified method

    Parameters
    ----------
    f : float
        Time value when you want to interpolate
    f1 : float
        Initial time value
    f2 : float
        Final time value
    x0 : float, optional
        Before initial position value
    x1 : float
        Initial position value
    x2 : float
        Final position value
    x3 : float, optional
        After final position value
    v1 : float, optional
        Initial velocity value
    v2 : float, optional
        Final velocity value
    tau : float, optional
        Value of Tau for Hermite
    method : str
        Interpolation method to use

    Returns
    -------
    float
        a float with the interpolated value
    '''

    # Relative position
    u = (f - f1) / (f2 - f1)

    if (method == 'LIN'):
        x  = interp_lin(u, x1, x2)
    elif (method == 'HER'):
        x = inter_hermite(u, x1, x2, v1, v2)
    elif (method == 'CAT'):
        x = inter_catmull(u, x0, x1, x2, x3, tau)
    return x

def get_pos (action, frm):
    '''Get the interpolated position of an object in a frame
       for each coordinate separately

    Parameters
    ----------
    action_name : str
        Name of the action on which it will be exectuted
    frm : float
        Frame to be interpolated

    Returns
    -------
    mathutils.Vector
        a mathutils.Vector with the 3 interpolated values (coordinates x, y, z)
    '''
    global obj, curve_pos, original_pos_curves

    scene_current_frame = bpy.context.scene.frame_current
    # To get the properties of the entered frame.
    bpy.context.scene.frame_set(frm)

    # Catmull-Rom tension
    tau = obj.tau
    # Selects if the users wants a random oscilation or not
    random_oscilation = bpy.data.objects[obj.name].oscilation
    # Maximun amplitude of the trajectory variation (if wanted)
    ampli_max = bpy.data.objects[obj.name].amplitude
    # Interpolation method to use
    interp_method = bpy.data.objects[obj.name].interpolation

    # Velocity curves
    curve_x_vel = action.fcurves.find('velocity', index = 0)
    curve_y_vel = action.fcurves.find('velocity', index = 1)
    curve_z_vel = action.fcurves.find('velocity', index = 2)
    curves_vel = [curve_x_vel, curve_y_vel, curve_z_vel]

    # Master action location curves
    master_action = bpy.data.actions[bpy.data.objects[obj.name].action_name]
    curve_x_pos = master_action.fcurves.find('location', index = 0)
    curve_y_pos = master_action.fcurves.find('location', index = 1)
    curve_z_pos = master_action.fcurves.find('location', index = 2)
    curves_pos = [curve_x_pos, curve_y_pos, curve_z_pos]

    # Final position after interpolation
    pos_coords = list()

    for curve_pos, curve_vel in zip(curves_pos, curves_vel):
        i = 0
        while ( (i < len(curve_pos.keyframe_points)) and (curve_pos.keyframe_points[i].co[0] < frm) ):
            i += 1
        # First keyframe position attribute of the scene
        pos_first_keyframe = curve_pos.keyframe_points[0].co[1]
        # First keyframe frame number attribute of the scene
        frm_first_keyframe = curve_pos.keyframe_points[0].co[0]
        # Last keyframe position attribute of the scene
        pos_last_keyframe = curve_pos.keyframe_points[-1].co[1]
        # First keyframe position attribute of the scene
        frm_last_keyframe = curve_pos.keyframe_points[-1].co[0]

        # Previous keyframe position attribute to the current frame (frm)
        pos_ini_keyframe = curve_pos.keyframe_points[i - 1].co[1]

        if (i == 0):
            pos_coords.append(pos_first_keyframe)

        elif (i == len(curve_pos.keyframe_points)):
            pos_coords.append(pos_ini_keyframe)

        else:
            # Next keyframe position attribute to the current frame (frm)
            pos_fin_keyframe = curve_pos.keyframe_points[i].co[1]
            # Previous keyframe frame number attribute to the current frame (frm)
            frm_ini_keyframe = curve_pos.keyframe_points[i - 1].co[0]
            # Next keyframe frame number attribute to the current frame (frm)
            frm_fin_keyframe = curve_pos.keyframe_points[i].co[0]

            if (bpy.data.objects[obj.name].interpolation == 'HER'):
                # Previous keyframe velocity attribute to the current frame (frm)
                vel_ini_keyframe = curve_vel.keyframe_points[i - 1].co[1]
                # Next keyframe velocity attribute to the current frame (frm)
                vel_fin_keyframe = curve_vel.keyframe_points[i].co[1]
            else:
                vel_ini_keyframe = 0
                vel_fin_keyframe = 0

            # Previous keyframe assignment
            if (frm_ini_keyframe <= frm_first_keyframe):
                previous_pos_keyframe = pos_ini_keyframe
            else:
                previous_pos_keyframe = curve_pos.keyframe_points[i - 2].co[1] # pos_ini_keyframe - 1

            # Next keyframe assignment
            if (frm_fin_keyframe >= frm_last_keyframe):
                next_pos_keyframe = pos_last_keyframe
            else:
                next_pos_keyframe = curve_pos.keyframe_points[i + 1].co[1] # pos_fin_keyframe + 1

            pos_coords.append( interpolate_values(frm,
                                                  frm_ini_keyframe,
                                                  frm_fin_keyframe,
                                                  previous_pos_keyframe,
                                                  pos_ini_keyframe,
                                                  pos_fin_keyframe,
                                                  next_pos_keyframe,
                                                  vel_ini_keyframe,
                                                  vel_fin_keyframe,
                                                  tau,
                                                  interp_method) )

    position_vector = mathutils.Vector(pos_coords)

    # If the user wants a random oscilation, we add it to the position vector
    # Moreover, we check if we are before the first keyframe or after the
    # last one for avoiding the original trajectory to be modified on those cases
    if(random_oscilation and (frm > frm_first_keyframe) and (frm < frm_last_keyframe)):
        position_vector += generate_mov_variation(ampli_max)

    # We put the scene frame we were before entering the function get_pos
    bpy.context.scene.frame_set(scene_current_frame)

    return position_vector

def generate_mov_variation (ampli_max):
    '''Gets the preferred maximum amplitude and generates
    a vector with 3 random numbers between -0.5 to -0.5 * max amplitude

    Parameters
    ----------
    ampli_max : float
        Value of the maximum amplitude

    Returns
    -------
    mathutils.Vector
        a mathutils.Vector with the 3 amplitude values (coordinates x, y, z)
    '''

    amplitude_values = [(random.random() - 0.5) * ampli_max, (random.random() - 0.5) * ampli_max, (random.random() - 0.5) * ampli_max]

    amplitude_vector = mathutils.Vector(amplitude_values)

    return amplitude_vector

def createEmptiesAndGetVelocityVector(action):
    '''
    Creates an empty arrow shaped object for each keyframe in the trajectory
    and fills it with a velocity vector that points to the next keyframe

    Parameters
    ----------
    action : bpy.data.en(curve_pos)actions
        Action of the object

    Returns
    -------
    list
        a list with mathutils.Vector-s that cointains the frame number and velocity keyframe values
        ex : [(25, [0,15,2]), ...]
    '''

    objName = obj.name
    # check if exists a collection with the same name of the action
    collection = bpy.data.collections.get(objName)

    if (collection):
        master_action = bpy.data.actions[bpy.data.objects[obj.name].action_name]
        curve_x_pos = master_action.fcurves.find('location', index = 0)
        if (len(collection.objects) is not len(curve_x_pos.keyframe_points)):
            cleanVelocityArrows(action)

    collection = bpy.data.collections.get(objName)

    if collection is None:
        # if the collection not exists then create it
        collection = bpy.data.collections.new(objName)
        bpy.context.scene.collection.children.link(collection)

    cx = action.fcurves.find('location',index=0).keyframe_points
    cy = action.fcurves.find('location',index=1).keyframe_points
    cz = action.fcurves.find('location',index=2).keyframe_points

    # for each keyframe check if exists an empty object in the collection
    for i in range(len(cx)):

        # get the position of the KF
        pos = mathutils.Vector([cx[i].co[1], cy[i].co[1], cz[i].co[1]])

        emptyObj = collection.objects.get(str(i))
        if emptyObj is None:
            # if not exists create it
            emptyObj = bpy.data.objects.new( "empty", None )
            emptyObj.empty_display_size = 2
            emptyObj.empty_display_type = 'SINGLE_ARROW'
            emptyObj.name = str(i)
            collection.objects.link(emptyObj)

            # else set the initial orientation to look at the next KFb
            if i+1 < len(cx):
                pos2 = mathutils.Vector([cx[i+1].co[1], cy[i+1].co[1], cz[i+1].co[1]])
                rot = (pos2 - pos).to_track_quat('Z','Y')
                emptyObj.rotation_euler = rot.to_euler()

        # set the location to the empty object
        emptyObj.location = mathutils.Vector([cx[i].co[1], cy[i].co[1], cz[i].co[1]])

    # force update the scene, necessary if new empties have been created
    bpy.context.view_layer.update()

    # for each keyframe get the velocity (scaled direction of the single arrow)
    # create a vector of tuples (frame, velocity) where velocity is a Vector
    velocityVector = []
    for i in range(len(cx)):
        velocity = collection.objects[str(i)].matrix_world.col[2][:3]
        velocityVector.append((cx[i].co[0], velocity))

    return velocityVector

def updateVelocityProperty(action, customVelocityPropertyName, velocityVector):
    '''
    Update the velocity values in the velocity vector
    (calculated by createEmptiesAndGetVelocityVector method) for each coordenate.

    Parameters
    ----------
    action : bpy.data.actions
        Action of the object
    customVelocityPropertyName : str
        Name of the velocity property
    velocityVector : list
        A list with mathutils.Vector-s that cointains the frame number and velocity keyframe values
    '''

    bpy.types.Object.velocity = bpy.props.FloatVectorProperty(name=customVelocityPropertyName)

    for i in range(3):

        # Remove the previous fcurve
        v = action.fcurves.find(customVelocityPropertyName, index=i)
        if v is not None:
            action.fcurves.remove(v)

        # create a clean new fcurve
        action.fcurves.new(customVelocityPropertyName, index=i)
        v = action.fcurves.find(customVelocityPropertyName, index=i)

        # add all the KFs to the fcurve. With extra values for more velocity.
        for kf in velocityVector:
            if (kf[1][i] > 0):
                v.keyframe_points.insert(kf[0], kf[1][i] + 2)
            elif (kf[1][i] < 0):
                v.keyframe_points.insert(kf[0], kf[1][i] - 2)
            else:
                v.keyframe_points.insert(kf[0], kf[1][i])

def cleanVelocityArrows(action):
    '''
    Removes the velocity arrows in the velocity vector when Hermite
    interpollation method is not selected

    Parameters
    ----------
    action : bpy.data.actions
        Action of the object
    '''

    # check if exists a collection with the same name of the action
    objName = obj.name

    collection = bpy.data.collections.get(objName)

    if collection is not None:
        cx = action.fcurves.find('location',index=0).keyframe_points

        # We are going to delete selected objects (so initially we didn't
        # select anything)
        bpy.ops.object.select_all(action='DESELECT')

        # for each keyframe check if exists an empty object in the collection
        for i in range(len(cx)):
            arrow = collection.objects.get(str(i))
            if arrow is not None:
                # if it exists we select it
                arrow.select_set(True)

        # we remove all the selected arrows
        bpy.ops.object.delete()

        # we unlink and remove the collection
        bpy.context.scene.collection.children.unlink(collection)
        bpy.data.collections.remove(collection)

def createTable(frm_start, frm_end, action):
    '''
    Create a table with the relation between the
    position of the frame and the distance traveled

    Parameters
    ----------
    action : bpy.data.actions
        Action of the object

    frm_start : float
        Frame in which the action starts

    frm_end : float
        Frame in which the action ends

    Returns
    -------
    list
        a list with the distance traveled for each frame position (list index)
    '''

    # For avoiding errors with the original trajectory, we
    # change the interpolation method to linear that doesn't
    # need any extra parameters
    #object_interpol = bpy.data.objects[obj.name].interpolation
    #bpy.data.objects[obj.name].interpolation = 'LIN'

    total_length = 0
    distance_list = [0]
    pos = get_pos(action, frm_start)

    for frm in range (frm_start + 1, frm_end + 1):
        pos_ant = pos
        pos = get_pos(action, frm)

        # distance difference between prev location
        # and current
        distance = (pos - pos_ant)
        # module of the vector (distance value)
        module = math.sqrt(distance.dot(distance))
        total_length = total_length + module

        # We round the length to 4 decimals
        distance_list.append(round(total_length,4))

    # We restore the original interpolation method
    #bpy.data.objects[obj.name].interpolation = object_interpol

    return distance_list

def binarySearch_table(table, value, frame_start):
    '''
    Searchs the value (longitude) on the table and
    returns the matching frame of the table.
    If the value is not on the table, interpolates
    the two consecutive frames.

    Parameters
    ----------
    table : list
        List with the distance traveled
        for each frame

    value : float
        Longitude we are looking for

    frm_start : float
        Frame in which the action starts

    Returns
    -------
    float
        value of the searched frame
    '''
    left = 0
    right = len(table) - 1
    center = (left + right) // 2

    value_rtrn = 0

    # Standard binary search
    while( (left <= right) and (table[center] is not value)):
        if(value < table[center]):
            right = center - 1
        else:
            left = center + 1
        center = (left + right) // 2

    if (value > table[len(table) - 1]):
        value_rtrn = len(table) - 1
    elif (value < table[0]):
        value_rtrn = 0
    elif (value <= table[center]): # interpolation of the 2 values obtained
        value_rtrn = interpolate_values(value, table[center -1], table[center],
                                        0, center, center - 1, 0, 0, 0, 0, 'LIN')
    else: #(value >= table[center])
        value_rtrn = interpolate_values(value, table[center], table[center + 1],
                                        0, center, center + 1, 0, 0, 0, 0, 'LIN')

    return (value_rtrn + frame_start)

def get_quat_from_vecs(original_orientation, final_orientation):
    '''Get the rotation quaternion that rotate the original vector to final vector.

    Parameters
    ----------
    original_orientation : mathutils.Vector
        Vector to orientate.
    final_orientation : mathutils.Vector
        Final vector orientation

    Returns
    -------
    mathutils.Quaternion
        the result rotation Quaternion
    '''
    # normalize() normaliza intrinsecamente. normalized() devuelve el vector normalizado.
    original_orientation.normalized()
    final_orientation.normalized()
    # Rotation axis and rotation angle
    rotation_axis = original_orientation.cross(final_orientation)
    rotation_axis.normalize()
    angle = math.acos(round(original_orientation.dot(final_orientation),4))

    cos_div_by_2 = math.cos(angle / 2)
    sin_div_by_2 = math.sin(angle / 2)

    rotation_quaternion = mathutils.Quaternion((cos_div_by_2,
                                    rotation_axis[0] * sin_div_by_2,
                                    rotation_axis[1] * sin_div_by_2,
                                    rotation_axis[2] * sin_div_by_2))

    return rotation_quaternion

def get_lat_vec(at):
    '''Get the horizontal vector that is perpendicular to "at" vector.

    Parameters
    ----------
    at : mathutils.Vector
        directional vector

    Returns
    -------
    mathutils.Vector
        the result perpendicular Vector
    '''
    at.normalize()
    up = mathutils.Vector((0.0, 0.0, 1.0))
    desired_lateral_orientation = up.cross(at)
    desired_lateral_orientation.normalize()
    return desired_lateral_orientation

def get_quat_rot(axis, tangent, lat_axis, alpha_inclination):
    '''Get the rotation quaternion that move the axis vector to tangent vector.

    Parameters
    ----------
    axis : mathutils.Vector
        vector to align
    tangent : mathutils.Vector
        tangent vector (final orientation)
    lat_axis : mathutils.Vector
        lateral axis to align
    alpha_inclination : Float
        Wrapping angle
    Returns
    -------
    mathutils.Quaternion
        the final rotation quaternion
    '''

    axis.normalize()
    tangent.normalize()
    lat_axis.normalize()

    rotation_axis = tangent # for lateral align

    #Aligns the main axis to the tangent of the trajectory
    q1 = get_quat_from_vecs (axis, tangent)
    #rot_quaternion1 = object1.rotation_quaternion.rotation_difference(object2.rotation_quaternion)

    lat_axis.rotate(q1)

    #Returns the final lateral orientation
    lateral_orientation = get_lat_vec(tangent)
    lateral_orientation.normalize()
    # Lateral oriantation after first quaternion rotation
    lat_axis.normalize()
    q2 = get_quat_from_vecs (lat_axis, lateral_orientation)

    cos_div_by_2 = math.cos(alpha_inclination / 2)
    sin_div_by_2 = math.sin(alpha_inclination / 2)

    #Wraping angle rotation quaternion
    q3 = mathutils.Quaternion((cos_div_by_2,
                                    rotation_axis[0] * sin_div_by_2,
                                    rotation_axis[1] * sin_div_by_2,
                                    rotation_axis[2] * sin_div_by_2))

    rot_quaternion = q3 @ q2 @ q1

    return rot_quaternion

def orientate_object(object, axis_to_align, vec, alpha_inclination, lat_axis_selected):
    '''Apply a tangent orientation to an object.

    Parameters
    ----------
    obj : object
        object to orientate
    axis_to_align : mathutils.Vector
        vector that will look at tangent
    vec : mathutils.Vector
        tangent vector
    alpha_inclination : Float
        Wrapping angle
    lat_axis_selected : char
        lateral axis selected
    '''
    #We get the vector to align to the tangent
    if (axis_to_align == 'X'):
        axis = mathutils.Vector((1.0, 0.0, 0.0))
    elif (axis_to_align == 'Y'):
        axis = mathutils.Vector((0.0, 1.0, 0.0))
    elif (axis_to_align == 'Z'):
        axis = mathutils.Vector((0.0, 0.0, 1.0))
    elif (axis_to_align == '-X'):
        axis = mathutils.Vector((-1.0, 0.0, 0.0))
    elif (axis_to_align == '-Y'):
        axis = mathutils.Vector((0.0, -1.0, 0.0))
    else:
        axis = mathutils.Vector((0.0, 0.0, -1.0))

    #We get the lateral vector
    if (lat_axis_selected == 'X'):
        lat_axis = mathutils.Vector((1.0, 0.0, 0.0))
    elif (lat_axis_selected == 'Y'):
        lat_axis = mathutils.Vector((0.0, 1.0, 0.0))
    elif (lat_axis_selected == 'Z'):
        lat_axis = mathutils.Vector((0.0, 0.0, 1.0))
    elif (lat_axis_selected == '-X'):
        lat_axis = mathutils.Vector((-1.0, 0.0, 0.0))
    elif (lat_axis_selected == '-Y'):
        lat_axis = mathutils.Vector((0.0, -1.0, 0.0))
    else:
        lat_axis = mathutils.Vector((0.0, 0.0, -1.0))

    q = get_quat_rot(axis, vec, lat_axis, alpha_inclination)

    object.rotation_quaternion = q

def get_tan (pos1, pos2):
    '''Return a vector by two points.

    Parameters
    ----------
    pos1 : Float
        first point
    pos2 : Float
        second point

    Returns
    -------
    mathutils.Vector
        result vector
    '''

    tan = mathutils.Vector((0.0, 0.0, 0.0))
    tan = pos2 - pos1
    tan.normalize()
    return tan

def calculate():
    '''
    Main function that is called by the operator that starts the interpolation
    between keyframes with all the custom properties values introduced by the user.

    '''
    global obj, action, curves_pos
    # Initial and final frame of the scene
    frame_start = bpy.context.scene.frame_start
    frame_end = bpy.context.scene.frame_end

    # Selected object
    obj = bpy.context.active_object
    obj.rotation_mode = 'QUATERNION'
    # Name of the action on which the script takes effect
    action_master = bpy.data.actions[bpy.data.objects[obj.name].action_name]

    # Copy the master action to asociate to the object.
    action = action_master.copy()

    # Avoid crash if animation data doesn't exist
    # Retrieved from 'Blender for Animation and
    # Film-Based Production', Manrique, M.
    obj.animation_data_create()
    obj.animation_data.action = action

    # Avoid errors if constant property is not selected
    # but apply_reparam was selected before (change it)
    if (not obj.velocity_settings):
        obj.apply_reparam = False
        obj.constant = False

    # Disable constant velocity if controlled velocity is selected.
    if (obj.apply_reparam):
        obj.constant = False

    # Disable controlled velocity if constant is selected.
    if (obj.constant):
        obj.apply_reparam = False

    # Insert Keyframe frecuency
    aux_frequency = bpy.data.objects[obj.name].frequency
    frequency = 50 + 1 - aux_frequency

    # If Hermite is selected, we create and add the velocity arrows
    if (bpy.data.objects[obj.name].interpolation == 'HER'):
        velocityVector = createEmptiesAndGetVelocityVector(action)
        updateVelocityProperty(action, 'velocity', velocityVector)

    # If Hermite is not selected we delete the arrows
    else:
        cleanVelocityArrows(action)

    # We create the distance table of the trajectory
    longitude_table = createTable(frame_start, frame_end, action)

    obj.keyframe_insert(data_path = 'location', frame = frame_start)
    # Removes old keyframes of the object action
    for frm in range(frame_start + 1, frame_end):
        #if (bpy.data.actions[obj.action_name].fcurves.find('location',index=0) is not None):
        obj.keyframe_delete('location', frame=frm)

    # Update the maximum distance value
    bpy.types.Object.distance = bpy.props.FloatProperty(min=0.0, max=longitude_table[-1])

    if(bpy.data.objects[obj.name].orientate):
        frequency = 1
        bpy.data.objects[obj.name].oscilation = False
    # Inserts location keyframes depending on the frequency selected
    # between the first and the last frame of the animation
    for frm in range(frame_start, frame_end, frequency):
        # sets the current frame
        bpy.context.scene.frame_set(frm)
        if (bpy.data.objects[obj.name].constant):
            time = frm/bpy.context.scene.render.fps # frm / 24
            desired_long = time * obj.velocity_value  # v = selected m/s
            desired_frm = binarySearch_table(longitude_table, desired_long, frame_start)
            # if it's chosen to modify the velocity by using distance
            # we use the obj distance property
        elif(bpy.data.objects[obj.name].apply_reparam):
            desired_long = obj.distance

            desired_frm = binarySearch_table(longitude_table, desired_long, frame_start)


        else:
            desired_frm = frm
        obj.location = get_pos(action, desired_frm)

        obj.keyframe_insert(data_path='location', frame = frm)

        # We manually insert a keyframe in the last frame just in case it has not
        # been inserted inside the loop
        if (bpy.data.objects[obj.name].constant == False):
            obj.location = get_pos(action, frame_end)
            obj.keyframe_insert(data_path = 'location', frame = frame_end)

        #if the user wants the object to get orientated
        if (bpy.data.objects[obj.name].orientate): #program on the interface
            axis = bpy.data.objects[obj.name].main_axis
            lat_axis = bpy.data.objects[obj.name].inclination_axis
            alpha = 0.0
            if (bpy.data.objects[obj.name].inclination):
                alpha = bpy.data.objects[obj.name].inclination_angle
                alpha = alpha * math.pi / 180.0

            first_keyframe_frame = action_master.fcurves.find('location', index = 0).keyframe_points[0].co[0]
            if (frm < first_keyframe_frame):
                tan = get_tan(get_pos(action, first_keyframe_frame), get_pos(action, first_keyframe_frame + 0.1))
                orientate_object(obj, axis, tan, alpha, lat_axis)

            elif (frm != frame_end): #For avoiding errors on the last frame
                if (get_pos(action, desired_frm) != get_pos(action, desired_frm + 0.1)):
                    tan = get_tan(get_pos(action, desired_frm), get_pos(action, desired_frm + 0.1))
                    orientate_object(obj, axis, tan, alpha, lat_axis)
            else:
                if (get_pos(action, desired_frm - 0.1) != get_pos(action, desired_frm)):
                        tan = get_tan(get_pos(action, desired_frm - 0.1), get_pos(action, desired_frm))
                        orientate_object(obj, axis, tan, alpha, lat_axis)

            obj.keyframe_insert(data_path = 'rotation_quaternion', frame = frm)


    # Location curves
    curve_x_pos = action.fcurves.find('location', index = 0)
    curve_y_pos = action.fcurves.find('location', index = 1)
    curve_z_pos = action.fcurves.find('location', index = 2)
    curves_pos = [curve_x_pos, curve_y_pos, curve_z_pos]

    # If constant velocity is selected, we change Blender
    # default interpolation (Bezier) and set it to Linear
    if (bpy.data.objects[obj.name].constant):
        for curve_pos in curves_pos:
            for kf in curve_pos.keyframe_points:
                kf.interpolation = 'LINEAR'
    else: # If it's not, we restored the  default value
        for curve_pos in curves_pos:
            for kf in curve_pos.keyframe_points:
                kf.interpolation = 'BEZIER'
