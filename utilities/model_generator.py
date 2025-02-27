import opensim as osim
import sys
import argparse
import os


class ModelGenerator:
    all_flags = [
        'ignore_activation_dynamics',
        'ignore_tendon_compliance',
        'remove_wrap_objects'
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

        # Load and initialize the model.
        osim.Logger.setLevelString('error')
        self.model = osim.Model(self.model_path)
        self.model.initSystem()
    
    @staticmethod
    def set_muscle_dynamics(model, model_name, 
                            ignore_activation_dynamics, 
                            ignore_tendon_compliance):
        muscles = model.updMuscles()
        for i in range(muscles.getSize()):
            muscle = muscles.get(i)
            muscle.set_ignore_activation_dynamics(ignore_activation_dynamics)
            muscle.set_ignore_tendon_compliance(ignore_tendon_compliance)

        if ignore_activation_dynamics and ignore_tendon_compliance:
            model_name += '_nomuscdyn'
        elif ignore_activation_dynamics and not ignore_tendon_compliance:
            model_name += '_tendyn'
        elif not ignore_activation_dynamics and ignore_tendon_compliance:
            model_name += '_actdyn'
        else:
            model_name += '_muscdyn'

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

    def generate_model(self, flags):
        model_name = f'{self.model_name}'

        model_name = self.set_muscle_dynamics(self.model, model_name, 
                flags['ignore_activation_dynamics'], 
                flags['ignore_tendon_compliance'])
        
        if flags['remove_wrap_objects']:
            model_name = self.remove_wrap_objects(self.model, model_name)

        self.model.printToXML(os.path.join(self.model_dir, f"{model_name}.osim"))

        print(f' --> Generated {model_name}.osim')

    def verify_flags(self):
        if self.flags_to_include:
            for flag in self.flags_to_include:
                if flag not in self.all_flags:
                    print(f"Error: '{flag}' is not a valid flag.")
                    sys.exit(1)

    def generate_models(self):
        # Verify the flags.
        self.verify_flags()

        self.flags_list = list()
        if not self.flags_to_include:
            self.flags_list.append({
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
                self.flags_list.append(flags)
            
        # Generate models for each combination of flags.
        for flags in self.flags_list:
            self.generate_model(flags)
