import opensim as osim
import sys
import argparse
import os


class ModelGenerator:
    all_flags = [
        'ignore_muscle_dynamics',
        'remove_wrap_objects',
        'disable_constraints'
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
    def set_muscle_dynamics(model, model_name, ignore_muscle_dynamics):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            muscle.set_ignore_activation_dynamics(ignore_muscle_dynamics)
            muscle.set_ignore_tendon_compliance(ignore_muscle_dynamics)

        if ignore_muscle_dynamics:
            model_name += '_nomuscdyn'

        return model_name

    @staticmethod
    def remove_wrap_objects(model, model_name):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            geometry_path = muscle.getGeometryPath()
            geometry_path.updWrapSet().clearAndDestroy()

        bodies = model.getBodySet()
        for i in range(bodies.getSize()):
            body = bodies.get(i)
            body.upd_WrapObjectSet().clearAndDestroy()

        model.finalizeConnections()
        model_name += '_nowrap'

        return model_name
    
    @staticmethod
    def disable_constraints(model, model_name):
        constraints = model.updConstraintSet()
        constraints.clearAndDestroy()

        model.finalizeConnections()
        model_name += '_noconstraints'

        return model_name

    def generate_model(self, flags, print_model=True):
        model = osim.Model(self.model_path)
        model.initSystem()
        model_name = f'{self.model_name}'

        model_name = self.set_muscle_dynamics(model, model_name, 
                                              flags['ignore_muscle_dynamics'])
        
        if flags['remove_wrap_objects']:
            model_name = self.remove_wrap_objects(model, model_name)

        if flags['disable_constraints']:
            model_name = self.disable_constraints(model, model_name)

        if print_model:
            model.printToXML(os.path.join(self.model_dir, f"{model_name}.osim"))
            print(f' --> Generated {model_name}.osim')

        return model_name

    def verify_flags(self):
        if self.flags_to_include:
            for flag in self.flags_to_include:
                if flag not in self.all_flags:
                    print(f"Error: '{flag}' is not a valid flag.")
                    sys.exit(1)

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
        for flags in flags_list:
            model_name = self.generate_model(flags, False)
            model_names.append(model_name)
        
        return model_names
