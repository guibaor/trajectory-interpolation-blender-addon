##############################################################################
# Animation Project 1
#
# Guillem Barceló Orts
# Ana Gloria Galvez Mellado
# Darío Rodríguez Hernández
# Bekzat Mukhamedali
#
##############################################################################
bl_info = {
    'name': 'Trajectory Effect Addon',
    'author': 'Ana Gloria Gálvez, Bekzat Mukhamedali, Darío Rodríguez, Guillem Barceló',
    'category': 'object',
    'version': (3, 0, 0),
    'blender': (2, 8, 0)
}

modulesNames = ['ui', 'interpolations_and_ops']

import sys
import importlib

modulesFullNames = {}
for currentModuleName in modulesNames:
    modulesFullNames[currentModuleName] = ('{}.{}'.format(__name__, currentModuleName))

for currentModuleFullName in modulesFullNames.values():
    if currentModuleFullName in sys.modules:
        importlib.reload(sys.modules[currentModuleFullName])
    else:
        globals()[currentModuleFullName] = importlib.import_module(currentModuleFullName)
        setattr(globals()[currentModuleFullName], 'modulesNames', modulesFullNames)

def register():
    for currentModuleName in modulesFullNames.values():
        if currentModuleName in sys.modules:
            if hasattr(sys.modules[currentModuleName], 'register'):
                sys.modules[currentModuleName].register()

def unregister():
    for currentModuleName in modulesFullNames.values():
        if currentModuleName in sys.modules:
            if hasattr(sys.modules[currentModuleName], 'unregister'):
                sys.modules[currentModuleName].unregister()

register()
