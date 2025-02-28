import os
from doit.action import CmdAction
from utilities.model_generator import ModelGenerator

######################################################################
#                            BASE TASKS                              #
######################################################################

class Task(object):
    """The derived class must set `self.name` somewhere within its constructor.
    The derived class must also have its own class variable `REGISTRY = []`.
    """
    REGISTRY = []
    def __init__(self):
        self.uptodate = []
        self.file_dep = []
        self.actions = []
        self.targets = []
        self.task_dep = []
        self.doc = None 
        self.title = None
        #self.clean = []
        # Add this specific instance to the class variable REGISTRY.
        self.REGISTRY.append(self)

    def add_action(self, file_dep, target, member_function, *args_to_member,
        **kwargs_to_member):
        """A convenient way to add an action to your task: file dependencies,
        targets, and the action's member function are grouped together. Call
        this within a derived class. The `file_dep` and `target` arguments are
        passed down to the `member_function`. So the signature of
        `member_function` should look like the following:

        ```

        def member_function(self, file_dep, target):
        ```
        
        Make sure the `target` option is not named `targets`; python-doit tries
        to be smart about actions with a `targets` parameter, and overrides the
        behavior we want here.
        

        The arguments `file_dep` and `target` should be lists or dicts.

        """
        if type(file_dep) == list:
            self.file_dep += file_dep
        else:
            self.file_dep += file_dep.values()
        if type(target) == list:
            self.targets += target
        else:
            self.targets += target.values()
        args = [file_dep, target]
        if len(args_to_member):
            args += args_to_member
        
        if bool(kwargs_to_member):
            self.actions.append((member_function, args, kwargs_to_member))
        else:
            self.actions.append((member_function, args))

    def copy_file(self, file_dep, target):
        """This can be used as the action for derived classes that want to copy
        a file from one place to another (e.g., from the source to the results
        directory).
        """
        import shutil
        to_dir = os.path.split(target[0])[0]
        if not os.path.exists(to_dir): os.makedirs(to_dir)
        shutil.copyfile(file_dep[0], target[0])

    @classmethod
    def create_doit_tasks(cls):
        """Create a specific task for each registered instance of this
        class.
        """
        # Python-doit invokes this function for any object in the `doit`
        # namespace that has it; this is how python-doit registers the tasks.
        for instance in cls.REGISTRY:
            yield {'basename': instance.name,
                    'file_dep': instance.file_dep,
                    'actions': instance.actions,
                    'targets': instance.targets,
                    'uptodate': instance.uptodate,
                    'task_dep': instance.task_dep,
                    'title': instance.title,
                    'doc': instance.doc,
                    #'clean': instance.clean,
                    } 

class StudyTask(Task):
    def __init__(self, study):
        super(StudyTask, self).__init__()
        self.study = study

class ModelTask(StudyTask):
    def __init__(self, model):
        super(ModelTask, self).__init__(model.study)
        self.model = model

class TestTask(ModelTask):
    def __init__(self, test):
        super(TestTask, self).__init__(test.model)
        self.test = test
            
######################################################################
#                           CUSTOM TASKS                             #
######################################################################

class TaskInstallDependencies(StudyTask):
    REGISTRY = []
    def __init__(self, study):
        super(TaskInstallDependencies, self).__init__(study)
        self.name = 'install_dependencies'
        
        self.install_script = os.path.join(self.study.config['dependencies_path'], 
                                           'install_dependencies.sh')
        self.opensim_cmd_exe = os.path.join(self.study.config['dependencies_path'], 
                                            'opensim', 'opensim_core_install', 'bin', 
                                            'opensim-cmd')    
        self.add_action([self.install_script], 
                        [self.opensim_cmd_exe], 
                        self.install_dependencies)

    def install_dependencies(self, file_dep, target):
        import subprocess
        p = subprocess.run(file_dep[0], check=True, shell=True,
                           cwd=os.path.dirname(file_dep[0]))
        if p.returncode != 0:
            raise Exception('Non-zero exit status: code %s.' % p.returncode)
        
class TaskGenerateModels(ModelTask):
    REGISTRY = []
    def __init__(self, model, flags):
        super(TaskGenerateModels, self).__init__(model)
        self.name = f'generate_models_{model.name}'
        self.flags = flags
        self.generator = ModelGenerator(model.path, flags)
        model_names = self.generator.generate_model_names()
        model_paths = [os.path.join(self.study.config['models_path'], model.name,
                       f'{model_name}.osim') for model_name in model_names]
        
        self.add_action([model.path], 
                        model_paths, 
                        self.generate_models)
        
    def generate_models(self, file_dep, target):
        self.generator.generate_models()

class TaskRunTest(TestTask):
    REGISTRY = []
    def __init__(self, test, flags):
        super(TaskRunTest, self).__init__(test)
        self.name = f'run_test_{test.name}'
        self.flags = flags

        self.add_action([test.model.path], 
                        [test.results_exp_path], 
                        self.run_test)

    def run_test(self, file_dep, target):
        pass