import os
import numpy as np
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

class BenchmarkTask(ModelTask):
    def __init__(self, benchmark):
        super(BenchmarkTask, self).__init__(benchmark.model)
        self.benchmark = benchmark
        self.exe_path =  os.path.join(self.study.config['benchmarks_path'], 
                                      benchmark.name)
            
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
        self.model_names, self.model_tags = self.generator.generate_model_names()
        self.model_paths = [os.path.join(self.study.config['models_path'], model.name,
                       f'{model_name}.osim') for model_name in self.model_names]
        
        self.add_action([model.path], 
                        self.model_paths, 
                        self.generate_models)
        
    def generate_models(self, file_dep, target):
        self.generator.generate_models()

class TaskRunBenchmark(BenchmarkTask):
    REGISTRY = []
    def __init__(self, benchmark, generate_models_task, exe_args=None):
        super(TaskRunBenchmark, self).__init__(benchmark)
        if exe_args is None:
            exe_args = dict()
        self.exe_args = exe_args
        self.benchmark_name = f'benchmark_{benchmark.name}'
        for arg in self.exe_args:
            self.benchmark_name  += f'_{arg}{exe_args[arg]}'
        self.benchmark_name  += f'_{benchmark.model.name}'
        self.name = f'run_{self.benchmark_name}'
        self.model_names = generate_models_task.model_names
        self.model_paths = generate_models_task.model_paths
        self.model_tags = generate_models_task.model_tags

        subdir = ''
        first = True
        for arg in self.exe_args:
            if first:
                subdir += f'{arg}{exe_args[arg]}'
                first = False
            else:
                subdir += f'_{arg}{exe_args[arg]}'

        if not subdir:
            subdir = 'default'

        self.result_path = os.path.join(benchmark.results_exp_path, subdir)
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.out_paths = [os.path.join(benchmark.results_exp_path, subdir, 
                                       f'{model_name}.json') 
                          for model_name in self.model_names]

        self.add_action(self.model_paths, 
                        self.out_paths, 
                        self.run_test)

    def run_test(self, file_dep, target):
        import subprocess
        
        for model_path, out_path in zip(file_dep, target):
            command = f'{self.exe_path} {model_path}'
            for arg in self.exe_args:
                command += f' --{arg} {self.exe_args[arg]}'
            command += f' --benchmark_out={out_path}'
            p = subprocess.run(command, check=True, shell=True,
                               cwd=self.benchmark.results_exp_path)
            if p.returncode != 0:
                raise Exception('Non-zero exit status: code %s.' % p.returncode)
            
class TaskPlotBenchmark(BenchmarkTask):
    REGISTRY = []
    def __init__(self, benchmark, run_task):
        super(TaskPlotBenchmark, self).__init__(benchmark)
        self.benchmark_name = run_task.benchmark_name
        self.name = f'plot_{self.benchmark_name}'

        self.model_names = run_task.model_names
        self.model_tags = run_task.model_tags
        self.out_paths = run_task.out_paths
        self.result_path = run_task.result_path
        self.cpu_times_path = os.path.join(self.result_path, 'cpu_times.png')

        self.add_action(self.out_paths, 
                        [self.cpu_times_path],
                        self.plot_benchmark)
        
    def plot_benchmark(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json
        
        # Initialize the dictionary of CPU times.
        cpu_times = dict()
        num_benchmarks = 0
        with open(file_dep[0]) as f:
            data = json.load(f)
            for benchmark in data['benchmarks']:
                cpu_times[benchmark['name']] = list()
            num_benchmarks = len(data['benchmarks'])
        colors = plt.cm.jet(range(num_benchmarks))

        # Fill the dictionary with CPU times.
        for out_path in file_dep:
            with open(out_path) as f:
                data = json.load(f)
                for benchmark in data['benchmarks']:
                    cpu_times[benchmark['name']].append(benchmark['cpu_time'])


        # Sort the CPU times and labels.
        sorted_indices = np.argsort(cpu_times[list(cpu_times.keys())[0]])
        labels = self.model_tags.copy()
        for key in cpu_times:
            cpu_times[key] = [cpu_times[key][i] for i in sorted_indices]
        labels = [labels[i] for i in sorted_indices]

        # Plot the CPU times.
        x = np.arange(len(labels))
        width = 0.6 / num_benchmarks
        multiplier = 0
        if num_benchmarks > 1:
            multiplier = -num_benchmarks // 2

        fig, ax = plt.subplots(figsize=(5, 8))
        
        for benchmark, cpu_time in cpu_times.items():
            offset = width * multiplier
            ax.barh(x + offset, cpu_time, width, label=benchmark)
            multiplier += 1

        all_xticks = [1e-2, 1e-1, 1, 10, 100, 1000, 10000, 100000]
        all_xticklabels = ['0.01 ms', '0.1 ms', '1 ms', '10 ms', '100 ms', 
                           '1 s', '10 s', '100 s']
        max_value = 0
        for key, value in cpu_times.items():
            max_value = max(max_value, max(value))

        xticks = list()
        xticklabels = list()
        for xtick, xticklabel in zip(all_xticks, all_xticklabels):
            if xtick < 100 * max_value:
                xticks.append(xtick)
                xticklabels.append(xticklabel)

        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(x, labels, fontsize=6)
        plt.xlabel(f'CPU Time')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)

        ax.set_axisbelow(True)
        plt.title(self.benchmark_name)
        plt.legend(loc='lower right', fontsize=6)

        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()
        