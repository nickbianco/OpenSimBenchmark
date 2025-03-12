import opensim as osim
import sys
import argparse
import os


class ModelGenerator:
    all_flags = [
        'ignore_activation_dynamics',
        'ignore_tendon_compliance',
        'remove_wrap_objects',
        'disable_constraints',
        'remove_muscles'
    ]

    def __init__(self, model_path, flags_to_include=None):
        # Verify the model path.
        if not os.path.exists(model_path):
            print(f"Error: The model file '{model_path}' does not exist.")
            sys.exit(1)
        if not model_path.endswith('.osim'):
            print(f"Error: The model file '{model_path}' is not a valid OpenSim model.")
            sys.exit(1)
        self.model_path = model_path
        self.flags_to_include = flags_to_include if flags_to_include else []

        # Store the model name and directory.
        self.model_name = os.path.basename(self.model_path).replace('.osim', '')
        self.model_dir = os.path.join(os.path.dirname(self.model_path), self.model_name)
        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir)

        # Disable logging.
        osim.Logger.setLevelString('error')

    @staticmethod
    def set_base_model(model):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            muscle.set_ignore_activation_dynamics(False)
            muscle.set_ignore_tendon_compliance(False)
    
    @staticmethod
    def ignore_activation_dynamics(model, model_name):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            muscle.set_ignore_activation_dynamics(True)

        model_name += '_noactdyn'
        tag = 'no act. dyn.'

        return model_name, tag
    
    @staticmethod
    def ignore_tendon_compliance(model, model_name):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            muscle.set_ignore_tendon_compliance(True)

        model_name += '_notendyn'
        tag = 'no ten. dyn.'

        return model_name, tag

    @staticmethod
    def remove_wrap_objects(model, model_name):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            geometry_path = muscle.getGeometryPath()
            geometry_path.updWrapSet().clearAndDestroy()
            # Remove all path points except the first and last.
            pathPoints = geometry_path.updPathPointSet()
            while pathPoints.getSize() > 2:
                pathPoints.remove(1)

        bodies = model.getBodySet()
        for i in range(bodies.getSize()):
            body = bodies.get(i)
            body.upd_WrapObjectSet().clearAndDestroy()

        model.finalizeConnections()
        model_name += '_nowrap'
        tag = 'no wrapping'

        return model_name, tag
    
    @staticmethod
    def remove_muscles(model, model_name):
        muscles = model.getMuscles()
        size = muscles.getSize()
        while size:
            muscle = muscles.get(0)
            index = model.getForceSet().getIndex(muscle, 0)
            model.updForceSet().remove(index)
            size = muscles.getSize()

        model.finalizeConnections()
        model_name += '_nomuscles'
        tag = 'no muscles'

        return model_name, tag
    
    @staticmethod
    def disable_constraints(model, model_name):
        constraints = model.updConstraintSet()
        constraints.clearAndDestroy()

        model.finalizeConnections()
        model_name += '_noconstraints'
        tag = 'no constraints'

        return model_name, tag

    def generate_model(self, flags, print_model=True):
        model = osim.Model(self.model_path)
        model.initSystem()
        model_name = f'{self.model_name}'
        tags = list()

        self.set_base_model(model)

        if flags.get('ignore_activation_dynamics'):
            model_name, tag = self.ignore_activation_dynamics(model, model_name)
            tags.append(tag)

        if flags.get('ignore_tendon_compliance'):
            model_name, tag = self.ignore_tendon_compliance(model, model_name)
            tags.append(tag)
        
        if flags.get('remove_wrap_objects'):
            model_name, tag = self.remove_wrap_objects(model, model_name)
            tags.append(tag)

        if flags.get('disable_constraints'):
            model_name, tag = self.disable_constraints(model, model_name)
            tags.append(tag)

        if flags.get('remove_muscles'):
            model_name, tag = self.remove_muscles(model, model_name)
            tags.append(tag)

        if print_model:
            model.printToXML(os.path.join(self.model_dir, f"{model_name}.osim"))
            print(f' --> Generated {model_name}.osim')

        tags.sort(key=len)
        tag = '\n'.join(tags)
        if not tag:
            tag = 'base model'

        return model_name, tag

    def verify_flags(self):
        if self.flags_to_include:
            for flag in self.flags_to_include:
                if flag not in self.all_flags:
                    print(f"Error: '{flag}' is not a valid flag.")
                    sys.exit(1)

    def filter_flags(self, flags):
        if 'remove_muscles' in flags and flags['remove_muscles']:
            if 'ignore_activation_dynamics' in flags and flags['ignore_activation_dynamics']:
                return False
            
            if 'ignore_tendon_compliance' in flags and flags['ignore_tendon_compliance']:
                return False
            
            if 'remove_wrap_objects' in flags and flags['remove_wrap_objects']:
                return False
            
        return True

    def get_flags_list(self):
        # Verify the user-specified flags.
        self.verify_flags()

        flags_list = list()
        if not self.flags_to_include:
            flags_list.append({
                flag: False for flag in self.all_flags
            })
        else:
            # Generate all combinations of flags for flags to include.
            from itertools import product
            combinations = list(product([True, False], 
                                        repeat=len(self.flags_to_include)))
            for combination in combinations:
                flags = {flag: combination[i] for i, 
                         flag in enumerate(self.flags_to_include)}
                
                if self.filter_flags(flags):
                    flags_list.append(flags)

        return flags_list

    def generate_models(self):
        # Generate models for each combination of flags.
        flags_list = self.get_flags_list()
        for flags in flags_list:
            self.generate_model(flags)

    def generate_model_names(self):
        # Generate model names for each combination of flags.
        flags_list = self.get_flags_list()
        model_names = list()
        model_tags = list()
        for flags in flags_list:
            model_name, model_tag = self.generate_model(flags, False)
            model_names.append(model_name)
            model_tags.append(model_tag)
        
        return model_names, model_tags
