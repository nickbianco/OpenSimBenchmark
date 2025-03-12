import os
import numpy as np
from doit.action import CmdAction
import shutil
import opensim as osim

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

class PerfTask(ModelTask):
    def __init__(self, perf):
        super(PerfTask, self).__init__(perf.model)
        self.perf = perf
        self.exe_path =  os.path.join(self.study.config['perfs_path'], 
                                      perf.name)
            
######################################################################
#                           CUSTOM TASKS                             #
######################################################################

def get_sub_directory(exe_args):
    subdir = ''

    if exe_args:
        first = True
        for arg in exe_args:
            if first:
                subdir += f'{arg}{exe_args[arg]}'
                first = False
            else:
                subdir += f'_{arg}{exe_args[arg]}'

    if not subdir:
        subdir = 'default'

    return subdir

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
        
class TaskCreateRajagopalModels(StudyTask):
    REGISTRY = []
    def __init__(self, study):
        super(TaskCreateRajagopalModels, self).__init__(study)
        self.name = 'create_rajagopal_models'
        self.base_model_path = os.path.join(self.study.config['data_path'],
                                             'Rajagopal', 'subject_walk_scaled.osim')
        self.function_based_paths = os.path.join(
                self.study.config['data_path'], 'Rajagopal', 
                'subject_walk_scaled_FunctionBasedPathSet.xml')
        
        self.model_names = ['Rajagopal', 'RajagopalPathActuators',
                            'RajagopalFunctionBasedPaths', 
                            'RajagopalFunctionBasedPathActuators',
                            'RajagopalDGF',
                            'RajagopalFunctionBasedPathsDGF']
        self.model_paths = list()
        for model_name in self.model_names:
            self.model_paths.append(os.path.join(self.study.config['models_path'], 
                                                 f'{model_name}.osim'))
    
        
        self.add_action([self.base_model_path, self.function_based_paths], 
                        self.model_paths, 
                        self.create_rajagopal_models)
        
    def create_rajagopal_models(self, file_dep, target):
        osim.Logger.setLevelString('error')
        
        # Base model.
        shutil.copyfile(file_dep[0], target[0])
        print(f' --> Created {target[0]}')

        # PathActuator model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithPathActuators())
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[1])
        print(f' --> Created {target[1]}')

        # Function-based path model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(file_dep[1]))
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[2])
        print(f' --> Created {target[2]}')

        # Function-based PathActuators model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithPathActuators())
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(file_dep[1]))
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[3])
        print(f' --> Created {target[3]}')

        # DeGrooteFregly2016Muscle model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[4])

        # Function-based DeGrooteFregly2016Muscle model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(file_dep[1]))
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[5])
        print(f' --> Created {target[5]}')

        
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
        self.benchmark_name = f'{benchmark.name}'
        for arg in self.exe_args:
            self.benchmark_name  += f'_{arg}{exe_args[arg]}'
        self.benchmark_name  += f'_{benchmark.model.name}'
        self.name = f'run_{self.benchmark_name}'
        self.model_names = generate_models_task.model_names
        self.model_paths = generate_models_task.model_paths
        self.model_tags = generate_models_task.model_tags
        self.subdir = get_sub_directory(self.exe_args)

        self.result_path = os.path.join(benchmark.results_path, self.subdir)
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.out_paths = [os.path.join(self.result_path, f'{model_name}.json') 
                          for model_name in self.model_names]
        
        self.add_action(self.model_paths,
                        self.out_paths, 
                        self.run_benchmark)

    def run_benchmark(self, file_dep, target):
        import subprocess
        import json
        for model_path, out_path in zip(file_dep, target):
            command = f'{self.exe_path} {model_path}'
            for arg in self.exe_args:
                command += f' --{arg} {self.exe_args[arg]}'
            command += f' --benchmark_out={out_path}'
            try:
                p = subprocess.run(command, check=True, shell=True,
                                    cwd=self.benchmark.results_path)
                if p.returncode != 0:
                    raise Exception(f'Non-zero exit status: code {p.returncode}.')
                
            except Exception as e:
                output = dict()
                output['failed'] = True
                output['error'] = str(e)
                with open(out_path, 'w') as f:
                    json.dump(output, f, indent=4)

            if not os.path.exists(out_path):
                output = dict()
                output['failed'] = True
                output['error'] = 'No output file was created.'
                with open(out_path, 'w') as f:
                    json.dump(output, f, indent=4) 
                
            
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

        # Filter out failed benchmark tests.
        model_tags = list()
        out_paths = list()
        for model_tag, out_path in zip(self.model_tags, file_dep):
            with open(out_path) as f:
                data = json.load(f)
                if 'failed' not in data:
                    model_tags.append(model_tag)
                    out_paths.append(out_path)
        
        # Initialize the dictionary of CPU times.
        cpu_times = dict()
        num_benchmarks = 0
        with open(out_paths[0]) as f:
            data = json.load(f)
            for benchmark in data['benchmarks']:
                cpu_times[benchmark['name']] = list()
            num_benchmarks = len(data['benchmarks'])

        # Fill the dictionary with CPU times.
        for out_path in out_paths:
            with open(out_path) as f:
                data = json.load(f)
                for benchmark in data['benchmarks']:
                    cpu_times[benchmark['name']].append(benchmark['cpu_time'])

        # Plot the CPU times.
        y = np.arange(len(model_tags))
        width = 0.6 / num_benchmarks
        multiplier = 0
        if num_benchmarks > 1:
            multiplier = -num_benchmarks // 2

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(5, 8))
        
        # Plot the CPU times.
        for benchmark, cpu_time in cpu_times.items():
            offset = width * multiplier
            ax.barh(y + offset, cpu_time, width, label=benchmark)
            multiplier += 1

        # Set the x-axis labels based on max CPU time.
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

        # Set the plot parameters.
        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(y, model_tags, fontsize=6)
        plt.xlabel(f'CPU Time')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.benchmark_name)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


class TaskRunPerf(PerfTask):
    REGISTRY = []
    def __init__(self, perf, event, generate_models_task, exe_args=None):
        super(TaskRunPerf, self).__init__(perf)
        if exe_args is None:
            exe_args = dict()
        self.exe_args = exe_args
        self.event = event
        self.perf_name = f'{perf.name}_{self.event}'
        for arg in self.exe_args:
            self.perf_name  += f'_{arg}{exe_args[arg]}'
        self.perf_name  += f'_{perf.model.name}'
        self.name = f'run_{self.perf_name}'
        self.model_names = generate_models_task.model_names
        self.model_paths = generate_models_task.model_paths
        self.model_tags = generate_models_task.model_tags
        self.subdir = get_sub_directory(self.exe_args)

        self.result_path = os.path.join(perf.results_path, self.subdir, self.event)
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.out_paths = [os.path.join(self.result_path, f'{model_name}.data') 
                          for model_name in self.model_names]
        self.json_paths = [os.path.join(self.result_path, f'{model_name}.json')
                           for model_name in self.model_names]

        self.add_action(self.model_paths, 
                        self.out_paths, 
                        self.run_perf)

    def run_perf(self, file_dep, target):
        import subprocess
        for model_path, out_path, json_path in zip(file_dep, target, self.json_paths):
            command = f'perf record -e {self.event} -F 997 -a -g'
            command += f' -o {out_path} {self.exe_path} {model_path} {json_path}'
            for arg in self.exe_args:
                command += f' --{arg} {self.exe_args[arg]}'
            try:
                p = subprocess.run(command, check=True, shell=True,
                                cwd=self.perf.results_path)
                if p.returncode != 0:
                    raise Exception('Non-zero exit status: code %s.' % p.returncode)
            except Exception as e:
                import json
                output = dict()
                output['failed'] = True
                output['error'] = str(e)
                with open(json_path, 'w') as f:
                    json.dump(output, f, indent=4)

            if not os.path.exists(out_path):
                output = dict()
                output['failed'] = True
                output['error'] = 'No output file was created.'
                with open(out_path, 'w') as f:
                    json.dump(output, f, indent=4) 
            

class TaskPlotPerf(PerfTask):
    REGISTRY = []
    def __init__(self, perf, run_task):
        super(TaskPlotPerf, self).__init__(perf)
        self.perf_name = run_task.perf_name
        self.name = f'plot_{self.perf_name}'

        self.model_names = run_task.model_names
        self.model_tags = run_task.model_tags
        self.json_paths = run_task.json_paths
        self.result_path = run_task.result_path
        self.time_elapsed_path = os.path.join(self.result_path, 'time_elapsed.png')
        self.num_steps_path = os.path.join(self.result_path, 'num_steps.png')
        self.step_size_path = os.path.join(self.result_path, 'step_size.png')

        self.add_action(self.json_paths, 
                        [self.time_elapsed_path],
                        self.plot_time_elapsed)
        
        if 'forward' in self.perf_name:
            self.add_action(self.json_paths, 
                            [self.num_steps_path],
                            self.plot_num_steps)
            
            self.add_action(self.json_paths,
                            [self.step_size_path],
                            self.plot_step_size)
        
        
    def plot_time_elapsed(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json

        # Filter out failed perf tests.
        json_paths = list()
        model_tags = list()
        for model_tag, json_path in zip(self.model_tags, file_dep):
            with open(json_path) as f:
                data = json.load(f)
                if 'failed' not in data:
                    model_tags.append(model_tag)
                    json_paths.append(json_path)
        
        time_elapsed = list()
        for json_path in json_paths:
            with open(json_path, 'rb') as f:
                data = json.load(f)
                time_elapsed.append(1000.0*data['time_elapsed'])

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(5, 8))
        
        # Plot time elapsed.
        y = np.arange(len(model_tags))
        ax.barh(y, time_elapsed, label='time elapsed')

        # Set the x-axis labels based on max CPU time.
        all_xticks = [1e-2, 1e-1, 1, 10, 100, 1000, 10000, 100000]
        all_xticklabels = ['0.01 ms', '0.1 ms', '1 ms', '10 ms', '100 ms', 
                           '1 s', '10 s', '100 s']
        max_value = max(time_elapsed)

        xticks = list()
        xticklabels = list()
        for xtick, xticklabel in zip(all_xticks, all_xticklabels):
            if xtick < 100 * max_value:
                xticks.append(xtick)
                xticklabels.append(xticklabel)

        # Set the plot parameters.
        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(y, model_tags, fontsize=6)
        plt.xlabel(f'Time Elapsed')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.perf_name)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


    def plot_num_steps(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json

        # Filter out failed perf tests.
        json_paths = list()
        model_tags = list()
        for model_tag, json_path in zip(self.model_tags, file_dep):
            with open(json_path) as f:
                data = json.load(f)
                if 'failed' not in data:
                    model_tags.append(model_tag)
                    json_paths.append(json_path)
    
        num_steps_taken = list()
        num_iterations = list()
        num_realizations = list()
        num_steps_attempted = list()
        num_error_test_failures = list()
        for out_path in file_dep:
            with open(out_path) as f:
                data = json.load(f)
                num_steps_taken.append(data['num_steps_taken'])
                num_iterations.append(data['num_iterations'])
                num_realizations.append(data['num_realizations'])
                num_steps_attempted.append(data['num_steps_attempted'])
                num_error_test_failures.append(data['num_error_test_failures'])

        # Plot the number of steps.
        y = np.arange(len(model_tags))
        width = 0.15
        multiplier = -2

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(5, 8))
        
        # Plot the step values.
        offset = width * multiplier
        ax.barh(y + offset, num_error_test_failures, width, 
                label='no. error test failures')

        offset = width * multiplier
        ax.barh(y + offset, num_steps_taken, width, label='no. steps taken')
        multiplier += 1

        offset = width * multiplier
        ax.barh(y + offset, num_iterations, width, label='no. iterations')
        multiplier += 1

        offset = width * multiplier
        ax.barh(y + offset, num_steps_attempted, width, label='no. steps attempted')
        multiplier += 1

        offset = width * multiplier
        ax.barh(y + offset, num_realizations, width, label='no. realizations')
        multiplier += 1

        # Set the x-axis labels based on max CPU time.
        all_xticks = [1, 10, 100, 1000, 10000, 100000]
        all_xticklabels = ['1', '10', '100', '1000', '10000', '100000']
        max_value = max(num_realizations)
        xticks = list()
        xticklabels = list()
        for xtick, xticklabel in zip(all_xticks, all_xticklabels):
            if xtick < 100 * max_value:
                xticks.append(xtick)
                xticklabels.append(xticklabel)

        # Set the plot parameters.
        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(y, model_tags, fontsize=6)
        plt.xlabel(f'Steps')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.perf_name)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


    def plot_step_size(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json

        # Filter out failed perf tests.
        json_paths = list()
        model_tags = list()
        for model_tag, json_path in zip(self.model_tags, file_dep):
            with open(json_path) as f:
                data = json.load(f)
                if 'failed' not in data:
                    model_tags.append(model_tag)
                    json_paths.append(json_path)
        
        initial_step_size = list()
        final_step_size = list()
        time_per_realization = list()
        for out_path in file_dep:
            with open(out_path) as f:
                data = json.load(f)
                initial_step_size.append(1000.0 * data['initial_step_size'])
                final_step_size.append(1000.0 * data['final_step_size'])
                time_per_realization.append(1000.0 * data['time_per_realization'])

       # Plot the step size.
        y = np.arange(len(model_tags))
        width = 0.2
        multiplier = -1

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(5, 8))
        
        # Plot the step sizes.
        offset = width * multiplier
        ax.barh(y + offset, initial_step_size, width, label='initial step size')
        multiplier += 1

        offset = width * multiplier
        ax.barh(y + offset, final_step_size, width, label='final step size')
        multiplier += 1

        offset = width * multiplier
        ax.barh(y + offset, time_per_realization, width, label='time per realization')
        multiplier += 1

        # Set the x-axis labels based on max CPU time.
        all_xticks = [1e-2, 1e-1, 1, 10, 100, 1000, 10000, 100000]
        all_xticklabels = ['0.01 ms', '0.1 ms', '1 ms', '10 ms', '100 ms', 
                           '1 s', '10 s', '100 s']
        max_value = max(time_per_realization)
        max_value = max(max_value, max(initial_step_size))
        max_value = max(max_value, max(final_step_size))

        xticks = list()
        xticklabels = list()
        for xtick, xticklabel in zip(all_xticks, all_xticklabels):
            if xtick < 100 * max_value:
                xticks.append(xtick)
                xticklabels.append(xticklabel)

        # Set the plot parameters.
        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(y, model_tags, fontsize=6)
        plt.xlabel(f'CPU Time')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.perf_name)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()
            

class TaskGenerateFlameGraph(PerfTask):
    REGISTRY = []
    def __init__(self, perf, run_task):
        super(TaskGenerateFlameGraph, self).__init__(perf)
        self.perf_name = run_task.perf_name
        self.name = f'generate_flamegraph_{self.perf_name}'

        self.model_names = run_task.model_names
        self.model_tags = run_task.model_tags
        self.json_paths = run_task.json_paths
        self.out_paths = run_task.out_paths
        self.result_path = run_task.result_path
        self.html_paths = [os.path.join(self.result_path, f'{model_name}.html') 
                          for model_name in self.model_names]

        self.add_action(self.out_paths, 
                        self.html_paths,
                        self.generate_flamegraph)
        
    def generate_flamegraph(self, file_dep, target):
        import json
        import subprocess

        # Filter out failed perf tests.
        out_paths = list()
        html_paths = list()
        for out_path, html_path, json_path in zip(file_dep, target, self.json_paths):
            with open(json_path) as f:
                data = json.load(f)
                if 'failed' not in data:
                    out_paths.append(out_path)
                    html_paths.append(html_path)

        # Generate the flamegraphs.
        for out_path, html_path in zip(out_paths, html_paths):
            command = f'cp {out_path} perf.data && '
            command += f'perf script report flamegraph --allow-download && '
            command += f'cp flamegraph.html {html_path} && '
            command += f'rm flamegraph.html perf.data'
            try: 
                p = subprocess.run(command, check=True, shell=True,
                                cwd=self.perf.results_path)
                if p.returncode != 0:
                    raise Exception('Non-zero exit status: code %s.' % p.returncode)
            except Exception as e:
                with open(html_path, 'w') as f:
                    f.write('')
                print(f'Error generating flamegraph: {e}')


class TaskGenerateDifferentialFlameGraph(PerfTask):
    REGISTRY = []
    def __init__(self, perf, run_task, model1, model2, negated=False):
        super(TaskGenerateDifferentialFlameGraph, self).__init__(perf)
        self.negated = negated
        self.negated_name = '_negated' if negated else ''
        self.negated_flag = '--negate' if negated else ''

        self.name = f'generate{self.negated_name}_differential_flamegraph_' \
                    f'{run_task.perf_name}_{model1}_vs_{model2}'
        
        
        self.result_path = run_task.result_path
        self.out_paths = [os.path.join(self.result_path, f'{model_name}.data') 
                          for model_name in [model1, model2]]

        self.html_path = os.path.join(self.result_path,
                                      f'{model1}_vs_{model2}{self.negated_name}.html')

        self.add_action(self.out_paths, 
                        [self.html_path],
                        self.generate_differential_flamegraph)
        
    def generate_differential_flamegraph(self, file_dep, target):
        import subprocess

        flamegraph = os.path.join(self.study.config['flamegraph_path'], 'flamegraph.pl')
        stackcollapse = os.path.join(self.study.config['flamegraph_path'], 
                                     'stackcollapse-perf.pl')
        difffolded = os.path.join(self.study.config['flamegraph_path'], 'difffolded.pl')

        if self.negated:
            first = file_dep[1]
            second = file_dep[0]
        else:
            first = file_dep[0]
            second = file_dep[1]

        command = f'perf script -i {first} > out.stacks1 && '
        command += f'perf script -i {second} > out.stacks2 && '
        command += f'{stackcollapse} out.stacks1 > out.folded1 && '
        command += f'{stackcollapse} out.stacks2 > out.folded2 && '
        command += f'{difffolded} -n out.folded1 out.folded2 | '
        command += f'{flamegraph} {self.negated_flag} > {target[0]} && '
        command += f'rm out.stacks1 out.stacks2 out.folded1 out.folded2'
        try:
            p = subprocess.run(command, check=True, shell=True,
                            cwd=self.perf.results_path)
            if p.returncode != 0:
                raise Exception('Non-zero exit status: code %s.' % p.returncode)
        except Exception as e:
            with open(target[0], 'w') as f:
                f.write('')
            print(f'Error generating differential flamegraph: {e}')
        

class TaskPlotBenchmarkComparison(StudyTask):
    REGISTRY = []
    def __init__(self, study, tag, model_names, benchmark, 
                 model_suffix='', exe_args=None):
        super(TaskPlotBenchmarkComparison, self).__init__(study)
        self.name = f'plot_benchmark_comparison_{tag}'
        self.tag = tag
        self.study = study
        self.model_names = model_names
        self.models = list()
        for model_name in model_names:
            model = self.study.get_model(model_name)
            if model is None:
                print(f' --> {self.name}: Model {model_name} not found.')
            else:
                self.models.append(model)
        self.model_suffix = model_suffix
        self.benchmark = benchmark
        self.exe_args = exe_args
        self.subdir = get_sub_directory(self.exe_args)
        self.results_path = self.study.config['results_path']
        self.figures_path = self.study.config['figures_path']
        
        self.plot_labels = list()
        self.json_paths = list()
        for model in self.models:
            self.json_paths.append(
                    os.path.join(self.results_path, model.name, self.benchmark,
                                 self.subdir, f'{model.name}{self.model_suffix}.json'))
            self.plot_labels.append(model.label)
            
        if not os.path.exists(self.figures_path):
            os.makedirs(self.figures_path)
        
        self.figure_path = os.path.join(self.figures_path, 
                                       f'benchmark_comparison_{tag}.png')

        self.add_action(self.json_paths, 
                        [self.figure_path], 
                        self.plot_benchmark_comparison)

    def plot_benchmark_comparison(self, file_dep, target):
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

        # Fill the dictionary with CPU times.
        for json_path in file_dep:
            with open(json_path) as f:
                data = json.load(f)
                for benchmark in data['benchmarks']:
                    cpu_times[benchmark['name']].append(benchmark['cpu_time'])

        # Plot the CPU times.
        y = np.arange(len(self.plot_labels))
        width = 0.6 / num_benchmarks
        multiplier = 0
        if num_benchmarks > 1:
            multiplier = -num_benchmarks // 2

        # Create the figure and axis.
        height = 1.0 * len(self.plot_labels)
        fig, ax = plt.subplots(figsize=(5, height))
        
        # Plot the CPU times.
        for benchmark, cpu_time in cpu_times.items():
            offset = width * multiplier
            ax.barh(y + offset, cpu_time, width, label=benchmark)
            multiplier += 1

        # Set the x-axis labels based on max CPU time.
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

        # Set the plot parameters.
        plt.xscale('log')
        plt.xticks(xticks, xticklabels)
        plt.yticks(y, self.plot_labels, fontsize=6)
        ax = plt.gca()
        for label in ax.get_yticklabels():
            label.set_va('center')
        plt.xlabel(f'CPU Time')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.benchmark)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()
