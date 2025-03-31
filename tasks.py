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

def get_result_name(result):
    result_name = ''
    units = ''
    if result == 'acceleration_compute_time':
        result_name = 'acceleration compute time'
        units = '(s)'
    elif result == 'single_step_time':
        result_name = 'single step time'
        units = '(s)'
    elif result == 'forward_integration_time':
        result_name = 'forward integration time'
        units = '(s)'
    elif result == 'real_time_factor':
        result_name = 'real time factor'
        units = ''
    elif result == 'energy_conservation':
        result_name = '$\\Delta$ total energy'
        units = '(J)'

    return result_name, units

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


class TaskMuJoCoPendulumBenchmark(StudyTask):
    REGISTRY = []
    def __init__(self, study, nlinks, step, time, integrator):
        super(TaskMuJoCoPendulumBenchmark, self).__init__(study)
        self.name = (f'run_mujoco_{nlinks}link_pendulum_benchmark'
                     f'_step{step}_time{time}_{integrator}')
        self.nlinks = nlinks
        self.step = step
        self.time = time
        self.integrator = integrator

        # A dummy file dependency to satisify doit's dependency tree.
        self.dummy_path = os.path.join(self.study.config['mujoco_path'], 'dummy.txt')
        self.result_path = os.path.join(self.study.config['results_path'], 'pendulums',
                'MuJoCo', f'{nlinks}link_pendulum')
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.result_file = os.path.join(self.result_path, 
                f'{nlinks}link_pendulum_step{step}_time{time}_{integrator}.json')
        
        self.add_action([self.dummy_path], 
                        [self.result_file], 
                        self.run_pendulum_benchmark)        

    def run_pendulum_benchmark(self, file_dep, target):
        import mujoco
        import numpy as np
        import time
        import json

        def reset_data(data):
            data.qpos[:] = 0
            data.qpos[0] = np.pi / 4.0
            data.qvel[:] = 0

        def create_n_link_pendulum(n_links):
            """
            Create an XML string for an N-link pendulum in MuJoCo.

            Args:
                n_links (int): Number of pendulum links.

            Returns:
                str: MuJoCo XML string for the N-link pendulum.
            """
            # XML header and simulation options
            xml = """<mujoco model="n_link_pendulum">

        <worldbody>
            <light pos="0 0 2"/>
            <body pos="0 0 0">
            <joint type="hinge" axis="0 1 0"/>
            <geom type="cylinder" size="0.02" fromto="0 -.02 0 0 .02 0"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 1.0 0 0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>
            """

            # Add links iteratively
            for i in range(1, n_links):
                xml += f"""
            <body pos="1.0 0 0">
                <joint type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.02" fromto="0 0 0 1.0 0 0"/>"""

            # Close all <body> tags
            xml += "\n" + "      </body>" *(n_links-1)

            # Close the worldbody and mujoco tags
            xml += """
            </body>
        </worldbody>
        </mujoco>"""

            model = mujoco.MjModel.from_xml_string(xml)
            model.opt.gravity = [0.0, 0.0, -9.81]
            return model

        model = create_n_link_pendulum(self.nlinks)
        data = mujoco.MjData(model)
        model.opt.timestep = self.step
        if self.integrator == 'Euler':
            model.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
        elif self.integrator == 'implicit':
            model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICIT
        elif self.integrator == 'implicitfast':
            model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        elif self.integrator == 'RK4':
            model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
        else:
            raise ValueError(f'Invalid integrator: {self.integrator}')

        # Benchmark 1: Joint acceleration computation
        acceleration_compute_times = list()
        for i in range(100):
            reset_data(data)
            start_time = time.time()
            mujoco.mj_forward(model, data)
            acceleration_compute_times.append(time.time() - start_time)
        acceleration_compute_time = np.mean(acceleration_compute_times)

        # Benchmark 2: Single simulation step
        single_step_times = list()
        for i in range(100):
            reset_data(data)
            start_time = time.time()
            mujoco.mj_step(model, data)
            single_step_times.append(time.time() - start_time)
        single_step_time = np.mean(single_step_times)

        # Benchmark 3: Forward simulation
        reset_data(data)
        mujoco.mj_forward(model, data)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        initial_energy = data.energy[0] + data.energy[1]

        forward_integration_times = list()
        real_time_factors = list()
        final_energies = list()
        for i in range(10):
            reset_data(data)
            start_time = time.time()
            total_steps = int(self.time / model.opt.timestep)
            for _ in range(total_steps):
                mujoco.mj_step(model, data)
            forward_integration_time = time.time() - start_time
            forward_integration_times.append(forward_integration_time)
            real_time_factors.append(self.time / forward_integration_time)
            mujoco.mj_energyPos(model, data)
            mujoco.mj_energyVel(model, data)
            final_energies.append(data.energy[0] + data.energy[1])
        energy_conservation = np.mean(final_energies) - initial_energy

        # Save the results to a JSON file.
        results = {
            'acceleration_compute_time': acceleration_compute_time,
            'single_step_time': single_step_time,
            'forward_integration_time': np.mean(forward_integration_times),
            'real_time_factor': np.mean(real_time_factors),
            'energy_conservation': energy_conservation
        }
        with open(target[0], 'w') as f:
            json.dump(results, f, indent=4)

class TaskMuJoCoPendulumBenchmarkRK4Custom(StudyTask):
    REGISTRY = []
    def __init__(self, study, nlinks, step, time):
        super(TaskMuJoCoPendulumBenchmarkRK4Custom, self).__init__(study)
        self.name = (f'run_mujoco_{nlinks}link_pendulum_benchmark'
                     f'_step{step}_time{time}_rk4_custom')
        self.nlinks = nlinks
        self.step = step
        self.time = time

        # A dummy file dependency to satisify doit's dependency tree.
        self.dummy_path = os.path.join(self.study.config['mujoco_path'], 'dummy.txt')
        self.result_path = os.path.join(self.study.config['results_path'], 'pendulums',
                'MuJoCo', f'{nlinks}link_pendulum')
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.result_file = os.path.join(self.result_path, 
                f'{nlinks}link_pendulum_step{step}_time{time}_rk4_custom.json')
        
        self.add_action([self.dummy_path], 
                        [self.result_file], 
                        self.run_pendulum_benchmark)        

    def run_pendulum_benchmark(self, file_dep, target):
        import mujoco
        import numpy as np
        import time
        import json

        def reset_data(data):
            data.qpos[:] = 0
            data.qpos[0] = np.pi / 4.0
            data.qvel[:] = 0

        def create_n_link_pendulum(n_links):
            """
            Create an XML string for an N-link pendulum in MuJoCo.

            Args:
                n_links (int): Number of pendulum links.

            Returns:
                str: MuJoCo XML string for the N-link pendulum.
            """
            # XML header and simulation options
            xml = """<mujoco model="n_link_pendulum">

        <worldbody>
            <light pos="0 0 2"/>
            <body pos="0 0 0">
            <joint type="hinge" axis="0 1 0"/>
            <geom type="cylinder" size="0.02" fromto="0 -.02 0 0 .02 0"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 1.0 0 0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>
            """

            # Add links iteratively
            for i in range(1, n_links):
                xml += f"""
            <body pos="1.0 0 0">
                <joint type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.02" fromto="0 0 0 1.0 0 0"/>"""

            # Close all <body> tags
            xml += "\n" + "      </body>" * (n_links-1)

            # Close the worldbody and mujoco tags
            xml += """
            </body>
        </worldbody>
        </mujoco>"""

            model = mujoco.MjModel.from_xml_string(xml)
            model.opt.gravity = [0.0, 0.0, -9.81]
            return model
        
        def get_y(data):
            return np.concatenate((data.qpos, data.qvel))

        def set_y(model, data, y):
            data.qpos[:] = y[:model.nq]
            data.qvel[:] = y[model.nq:model.nq + model.nv]    

        def get_ydot(data):
            return np.concatenate((data.qvel, data.qacc))

        def set_state_and_realize_derivatives(model, data, t, y):
            data.time = t
            data.qpos[:] = y[:model.nq]
            data.qvel[:] = y[model.nq:model.nq + model.nv]
            mujoco.mj_forward(model, data)

        def take_rk4_step(model, data):
            """
            0  |
            1/2 | 1/2
            1/2 |  0  1/2
            1  |  0   0   1
            --------------------
                | 1/6 2/6 2/6 1/6
            """

            dt = model.opt.timestep
            t0 = data.time
            y0 = get_y(data)
            f0 = get_ydot(data)
            k1 = f0

            set_state_and_realize_derivatives(model, data, t0 + 0.5*dt, y0 + dt*0.5*k1)
            k2 = get_ydot(data)

            set_state_and_realize_derivatives(model, data, t0 + 0.5*dt, y0 + dt*0.5*k2)
            k3 = get_ydot(data)

            set_state_and_realize_derivatives(model, data, t0 + dt, y0 + dt*k3)
            k4 = get_ydot(data)

            y1 = y0 + dt*(1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4)
            set_y(model, data, y1)
            set_state_and_realize_derivatives(model, data, t0 + dt, y1)

        model = create_n_link_pendulum(self.nlinks)
        data = mujoco.MjData(model)
        model.opt.timestep = self.step

        # Initial energy
        reset_data(data)
        mujoco.mj_forward(model, data)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        initial_energy = data.energy[0] + data.energy[1]
        print('Initial energy:', initial_energy)

        final_energies = list()
        for i in range(10):
            reset_data(data)
            total_steps = int(self.time / model.opt.timestep)
            for _ in range(total_steps):
                take_rk4_step(model, data)
            mujoco.mj_forward(model, data)
            mujoco.mj_energyPos(model, data)
            mujoco.mj_energyVel(model, data)
            final_energies.append(data.energy[0] + data.energy[1])
        final_energy = np.mean(final_energies)
        print('Final energy:', final_energy)
        energy_conservation = final_energy - initial_energy
        print('Energy conservation:', energy_conservation)
        percent_change = 0
        if initial_energy != 0:
            percent_change = (energy_conservation / initial_energy) * 100
        print('Percent change:', percent_change)

        # Save the results to a JSON file.
        results = {
            'initial_energy': initial_energy,
            'final_energy': final_energy,
            'energy_conservation': energy_conservation,
            'percent_change': percent_change
        }
        with open(target[0], 'w') as f:
            json.dump(results, f, indent=4)

class TaskSimbodyPendulumBenchmark(StudyTask):
    REGISTRY = []
    def __init__(self, study, nlinks, step, time, integrator):
        super(TaskSimbodyPendulumBenchmark, self).__init__(study)
        self.name = (f'run_simbody_{nlinks}link_pendulum_benchmark'
                     f'_step{step}_time{time}_{integrator}')
        self.nlinks = nlinks
        self.step = step
        self.time = time
        self.integrator = integrator
        self.exe_path =  os.path.join(self.study.config['benchmarks_path'], 
                f'benchmark_simbody_pendulum_{integrator}')

        # A dummy file dependency to satisify doit's dependency tree.
        self.dummy_path = os.path.join(self.study.config['data_path'], 'pendulum',
                                       'dummy.txt')
        self.result_path = os.path.join(self.study.config['results_path'], 'pendulums',
                'Simbody', f'{nlinks}link_pendulum')
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.result_file = os.path.join(self.result_path, 
                f'{nlinks}link_pendulum_step{step}_time{time}_{integrator}.json')
        
        self.add_action([self.dummy_path], 
                        [self.result_file], 
                        self.run_pendulum_benchmark)        

    def run_pendulum_benchmark(self, file_dep, target):
        import subprocess
        command = f'{self.exe_path} {self.nlinks} {target[0]}'
        command += f' --time {self.time} --step {self.step}'
        try:
            p = subprocess.run(command, check=True, shell=True,
                            cwd=self.result_path)
            if p.returncode != 0:
                raise Exception('Non-zero exit status: code %s.' % p.returncode)
        except Exception as e:
            import json
            output = dict()
            output['failed'] = True
            output['error'] = str(e)
            with open(target[0], 'w') as f:
                json.dump(output, f, indent=4)

        if not os.path.exists(target[0]):
            output = dict()
            output['failed'] = True
            output['error'] = 'No output file was created.'
            with open(target[0], 'w') as f:
                json.dump(output, f, indent=4) 


class TaskSimbodyPendulumBenchmarkRK4Custom(StudyTask):
    REGISTRY = []
    def __init__(self, study, nlinks, step, time):
        super(TaskSimbodyPendulumBenchmarkRK4Custom, self).__init__(study)
        self.name = (f'run_simbody_{nlinks}link_pendulum_benchmark'
                     f'_step{step}_time{time}_rk4_custom')
        self.nlinks = nlinks
        self.step = step
        self.time = time
        self.exe_path =  os.path.join(self.study.config['benchmarks_path'], 
                f'benchmark_simbody_pendulum_rk4_custom')

        # A dummy file dependency to satisify doit's dependency tree.
        self.dummy_path = os.path.join(self.study.config['data_path'], 'pendulum',
                                       'dummy.txt')
        self.result_path = os.path.join(self.study.config['results_path'], 'pendulums',
                'Simbody', f'{nlinks}link_pendulum')
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.result_file = os.path.join(self.result_path, 
                f'{nlinks}link_pendulum_step{step}_time{time}_rk4_custom.json')
        
        self.add_action([self.dummy_path], 
                        [self.result_file], 
                        self.run_pendulum_benchmark)        

    def run_pendulum_benchmark(self, file_dep, target):
        import subprocess
        command = f'{self.exe_path} {self.nlinks} {target[0]}'
        command += f' --time {self.time} --step {self.step}'
        try:
            p = subprocess.run(command, check=True, shell=True,
                            cwd=self.result_path)
            if p.returncode != 0:
                raise Exception('Non-zero exit status: code %s.' % p.returncode)
        except Exception as e:
            import json
            output = dict()
            output['failed'] = True
            output['error'] = str(e)
            with open(target[0], 'w') as f:
                json.dump(output, f, indent=4)

        if not os.path.exists(target[0]):
            output = dict()
            output['failed'] = True
            output['error'] = 'No output file was created.'
            with open(target[0], 'w') as f:
                json.dump(output, f, indent=4) 
        

class TaskPlotSimbodyVsMuJoCoSpeedAccuracyTradeoff(StudyTask):
    REGISTRY = []
    def __init__(self, study, nlinks, steps, time, integrator):
        super(TaskPlotSimbodyVsMuJoCoSpeedAccuracyTradeoff, self).__init__(study)
        self.name = f'plot_simbody_vs_mujoco_speed_accuracy_tradeoff_time{time}_{integrator}'
        self.nlinks = nlinks
        self.steps = steps
        self.time = time
        self.integrator = integrator
        self.integrator_name = ''
        if self.integrator.lower() == 'euler':
            self.integrator_name = 'semi-explicit Euler'
        elif self.integrator.lower() == 'rk4':
            self.integrator_name = 'Runge-Kutta 4th order'

        self.mujoco_result_files = list()
        self.simbody_result_files = list()
        for nlink in self.nlinks:
            for step in self.steps:
                self.mujoco_result_files.append(
                        os.path.join(self.study.config['results_path'], 
                        'pendulums', 'MuJoCo', f'{nlink}link_pendulum',
                        f'{nlink}link_pendulum_step{step}_time{time}_{integrator}.json'))
                self.simbody_result_files.append(
                        os.path.join(self.study.config['results_path'], 
                        'pendulums', 'Simbody', f'{nlink}link_pendulum',
                        f'{nlink}link_pendulum_step{step}_time{time}_{integrator.lower()}.json'))
                
        self.figure_path = os.path.join(self.study.config['figures_path'],
                f'speed_accuracy_tradeoff_time{time}_{integrator}.png')
        
        self.add_action(self.mujoco_result_files + self.simbody_result_files, 
                        [self.figure_path],
                        self.plot_speed_accuracy_tradeoff)

    def plot_speed_accuracy_tradeoff(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json

        mujoco_energies = list()
        mujoco_real_time_factors = list()
        simbody_energies = list()
        simbody_real_time_factors = list()

        for mujoco_file, simbody_file in zip(self.mujoco_result_files, 
                                             self.simbody_result_files):
            with open(mujoco_file) as f:
                data = json.load(f)
                mujoco_energies.append(abs(data['energy_conservation']))
                mujoco_real_time_factors.append(data['real_time_factor'])

            with open(simbody_file) as f:
                data = json.load(f)
                simbody_energies.append(abs(data['energy_conservation']))
                simbody_real_time_factors.append(data['real_time_factor'])

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(4, 4))

        # Plot a scatter plot of the speed-accuracy tradeoff.
        ax.scatter(mujoco_energies, mujoco_real_time_factors, color='darkorange',
                   label='MuJoCo')
        ax.scatter(simbody_energies, simbody_real_time_factors, color='blue',
                   label='Simbody')
        
        # Set the plot parameters.
        plt.xscale('log')
        plt.yscale('log')
        plt.xlabel('$\\Delta$ total energy (J)')
        plt.ylabel('real time factor')
        plt.title(f'{self.integrator_name}')
        plt.legend(fontsize=8)
        plt.grid(zorder=0)
        ax = plt.gca()
        ax.set_axisbelow(True)

        # Save the figure.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


class TaskAggregatePendulumResults(StudyTask):
    REGISTRY = []
    def __init__(self, study, engine, nlinks, steps, time, integrators):
        super(TaskAggregatePendulumResults, self).__init__(study)
        self.name = f'aggregate_{engine.lower()}_pendulum_results_time{time}'
        self.engine = engine
        self.nlinks = nlinks
        self.steps = steps
        self.time = time
        self.integrators = integrators
        self.result_types = ['acceleration_compute_time', 'single_step_time',
                             'forward_integration_time', 'real_time_factor',
                             'energy_conservation']

        for integrator in self.integrators:
            result_files = list()
            for nlink in self.nlinks:
                for step in self.steps:
                    result_files.append(os.path.join(self.study.config['results_path'], 
                        'pendulums', engine, f'{nlink}link_pendulum',
                        f'{nlink}link_pendulum_step{step}_time{time}_{integrator}.json'))
                
            csv_paths = list()
            for result_type in self.result_types:
                csv_path = os.path.join(self.study.config['results_path'], 'pendulums',
                        engine, f'pendulum_{result_type}_time{time}_{integrator}.csv')
                csv_paths.append(csv_path)
        
            self.add_action(result_files, 
                            csv_paths,
                            self.aggregate_pendulum_results, integrator)

    def aggregate_pendulum_results(self, file_dep, target, integrator):
        import matplotlib.pyplot as plt
        import json
        import csv
        
        # Create CSV files for each result type where the number of pendulum links
        # are along the rows and the performance results are columns. Use different columns
        # for different step sizes.

        # Initialize the dictionary of results.
        results = dict()
        for result_type in self.result_types:
            results[result_type] = dict()
            for nlink in self.nlinks:
                results[result_type][nlink] = dict()
                for step in self.steps:
                    results[result_type][nlink] = list()
        
        # Read the JSON files and fill the results dictionary.
        result_idx = 0
        for nlink in self.nlinks:
            for step in self.steps:
                with open(file_dep[result_idx]) as f:
                    data = json.load(f)
                    for result_type in self.result_types:
                        if result_type in data:
                            results[result_type][nlink].append(data[result_type])
                result_idx += 1
       
        # Create the CSV files.
        for result_type in self.result_types:
            csv_path = target[self.result_types.index(result_type)]
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write the header row.
                header = ['nlinks'] + [f'step{step}' for step in self.steps]
                writer.writerow(header)
                # Write the data rows.
                for nlink in self.nlinks:
                    row = [nlink] + results[result_type][nlink]
                    writer.writerow(row)


class TaskPlotPendulumComparison(StudyTask):
    REGISTRY = []
    def __init__(self, study, engines, result, nlinks, step, time, integrator):
        super(TaskPlotPendulumComparison, self).__init__(study)
        self.name = f'plot_pendulum_comparison_{result}_time{time}_step{step}_{integrator}'
        self.result = result
        self.engines = engines
        self.nlinks = nlinks
        self.step = step
        self.time = time
        self.integrator = integrator
        self.integrator_name = ''
        if self.integrator.lower() == 'euler':
            self.integrator_name = 'semi-explicit Euler'
        elif self.integrator.lower() == 'rk4':
            self.integrator_name = 'Runge-Kutta 4th order'

        self.result_name, self.units = get_result_name(self.result)

        self.aggregate_files = list()
        for engine in engines:
            self.aggregate_files.append(
                    os.path.join(self.study.config['results_path'],
                    'pendulums', engine, 
                    f'pendulum_{result}_time{time}_{integrator}.csv'))
            
        self.figure_path = os.path.join(self.study.config['figures_path'],
                f'pendulum_{result}_comparison_time{time}_step{step}_{integrator}.png')
        
        self.add_action(self.aggregate_files, 
                        [self.figure_path],
                        self.plot_pendulum_comparison)

    def plot_pendulum_comparison(self, file_dep, target):
        import matplotlib.pyplot as plt

        import pandas as pd
        # Read the CSV files into dataframes.
        dataframes = []
        for file in file_dep:
            df = pd.read_csv(file)
            dataframes.append(df)

        # Create a grouped bar chart, where the bars are along the 
        # x-axis and the performance results are along the y-axis.
        # Group by 'engine'. Select the data column based on 'step'.

        fig, ax = plt.subplots(figsize=(4, 4))
        bar_width = 0.25
        bar_positions = np.arange(len(self.nlinks))
        for i, df in enumerate(dataframes):
            # Select the data column based on 'step'.
            step_idx = df.columns.get_loc(f'step{self.step}')
            data = abs(df.iloc[:, step_idx])
            ax.bar(bar_positions + i * bar_width, data, 
                   width=bar_width, label=df.columns[0])    
            
        # Plot the y-axis on a log scale
        ax.set_yscale('log')

        # Set the x-ticks and labels.
        ax.set_xticks(bar_positions + bar_width / 2)
        ax.set_xticklabels(self.nlinks)

        # Set the y-label and title.
        ax.set_xlabel('# of pendulum links')
        ax.set_ylabel(f'{self.result_name} {self.units}')
        if 'acceleration_compute_time' not in self.result:
            ax.set_title(f'{self.integrator_name} with $\\Delta t$ = {self.step} s')

        # Set the legend based on the engine names in self.engines  
        ax.legend(self.engines, fontsize=8)

        # Set the grid include minor ticks.
        ax.grid(which='major', linestyle='-', linewidth=0.5)
        ax.grid(which='minor', linestyle='-', linewidth=0.2, alpha=0.5)


        ax = plt.gca()
        ax.set_axisbelow(True)
        # Save the figure.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


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
        
        if not os.path.exists(self.study.config['models_path']):
            os.makedirs(self.study.config['models_path'])
        
        self.model_names = ['Rajagopal', 
                            'RajagopalPathActuators',
                            'RajagopalFunctionBasedPaths',
                            'RajagopalFunctionBasedPathsNoConstraints', 
                            'RajagopalFunctionBasedPathActuators',
                            'RajagopalFunctionBasedPathActuatorsNoConstraints', 
                            'RajagopalDGF',
                            'RajagopalFunctionBasedPathsDGF',
                            'RajagopalFunctionBasedPathsDGFNoConstraints']
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

        # Function-based path model without constraints.
        constraints = model.updConstraintSet()
        constraints.clearAndDestroy()

        bodyset = model.updBodySet()
        patella_l = bodyset.get('patella_l')
        patella_r = bodyset.get('patella_r')
        bodyset.remove(patella_l)
        bodyset.remove(patella_r)

        jointset = model.updJointSet()
        patellofemoral_l = jointset.get('patellofemoral_l')
        patellofemoral_r = jointset.get('patellofemoral_r')
        jointset.remove(patellofemoral_l)
        jointset.remove(patellofemoral_r)

        model.finalizeConnections()
        model.printToXML(target[3])
        print(f' --> Created {target[3]}')

        # Function-based PathActuators model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithPathActuators())
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(file_dep[1]))
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[4])
        print(f' --> Created {target[4]}')

        # Function-based PathActuators model without constraints.
        constraints = model.updConstraintSet()
        constraints.clearAndDestroy()

        bodyset = model.updBodySet()
        patella_l = bodyset.get('patella_l')
        patella_r = bodyset.get('patella_r')
        bodyset.remove(patella_l)
        bodyset.remove(patella_r)

        jointset = model.updJointSet()
        patellofemoral_l = jointset.get('patellofemoral_l')
        patellofemoral_r = jointset.get('patellofemoral_r')
        jointset.remove(patellofemoral_l)
        jointset.remove(patellofemoral_r)

        model.finalizeConnections()
        model.printToXML(target[5])
        print(f' --> Created {target[5]}')

        # DeGrooteFregly2016Muscle model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[6])

        # Function-based DeGrooteFregly2016Muscle model.
        modelProcessor = osim.ModelProcessor(file_dep[0])
        modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(file_dep[1]))
        model = modelProcessor.process()
        model.finalizeConnections()
        model.initSystem()
        model.printToXML(target[7])
        print(f' --> Created {target[7]}')

        # Function-based DeGrooteFregly2016Muscle model without constraints.
        constraints = model.updConstraintSet()
        constraints.clearAndDestroy()

        bodyset = model.updBodySet()
        patella_l = bodyset.get('patella_l')
        patella_r = bodyset.get('patella_r')
        bodyset.remove(patella_l)
        bodyset.remove(patella_r)

        jointset = model.updJointSet()
        patellofemoral_l = jointset.get('patellofemoral_l')
        patellofemoral_r = jointset.get('patellofemoral_r')
        jointset.remove(patellofemoral_l)
        jointset.remove(patellofemoral_r)

        model.finalizeConnections()
        model.printToXML(target[8])
        print(f' --> Created {target[8]}')

        
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
            command = f'{self.exe_path} {model_path} {out_path}'
            for arg in self.exe_args:
                command += f' --{arg} {self.exe_args[arg]}'
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
        self.time = run_task.exe_args['time'] if 'time' in run_task.exe_args else None
        self.result_path = run_task.result_path
        self.results = ['acceleration_compute_time', 'single_step_time',
                        'forward_integration_time', 'real_time_factor',
                        'energy_conservation']
        self.result_names = list()
        self.units_list = list()
        for result in self.results:
            result_name, units = get_result_name(result)
            self.result_names.append(result_name)
            self.units_list.append(units)

        self.result_paths = list()
        for result in self.results:
            result_path = os.path.join(self.result_path, f'{result}.png')
            self.result_paths.append(result_path)
        
        self.add_action(self.out_paths, 
                        self.result_paths,
                        self.plot_benchmark)

        
    def plot_benchmark(self, file_dep, target):
        import matplotlib.pyplot as plt
        import json
        
        # Initialize the dictionary of CPU times.
        results_dict = dict()
        for result in self.results:
            results_dict[result] = np.zeros(len(self.model_tags))
        for i, out_path in enumerate(file_dep):
            with open(out_path) as f:
                data = json.load(f)
                for result in self.results:
                    if result in data:
                        results_dict[result][i] = data[result]

        zipped = zip(self.results, self.result_names, self.units_list, target)
        for i, (result, result_name, units, result_path) in enumerate(zipped):
            
            fig, ax = plt.subplots(figsize=(4, 4))
            bar_width = 0.25
            x = np.arange(len(self.model_tags))
            ax.bar(x, abs(results_dict[result]), width=bar_width)    
                
            # Plot the y-axis on a log scale
            ax.set_yscale('log')

            # Set the x-ticks and labels. Use 45 degree rotation for the labels.
            ax.set_xticks(x)
            ax.set_xticklabels(self.model_tags, rotation=45, ha='right')
            
            # Set the y-label and title.
            ax.set_ylabel(f'{result_name} {units}')

            # Set the grid include minor ticks.
            ax.grid(which='major', linestyle='-', linewidth=0.5)
            ax.grid(which='minor', linestyle='-', linewidth=0.2, alpha=0.5)

            if 'real_time_factor' in result:
                ax.axhline(y=1, color='red', linestyle='-', lw=2, zorder=0)

            ax = plt.gca()
            ax.set_axisbelow(True)
            # Save the figure.
            plt.tight_layout()
            plt.savefig(result_path, dpi=600)
            plt.close()


    def plot_real_time_factor(self, file_dep, target):
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
        real_time_factors = dict()
        num_benchmarks = 0
        with open(out_paths[0]) as f:
            data = json.load(f)
            for benchmark in data['benchmarks']:
                real_time_factors[benchmark['name']] = list()
            num_benchmarks = len(data['benchmarks'])

        # Fill the dictionary with CPU times.
        for out_path in out_paths:
            with open(out_path) as f:
                data = json.load(f)
                for benchmark in data['benchmarks']:
                    real_time_factors[benchmark['name']].append(
                        self.time / (0.001*benchmark['cpu_time']))

        # Plot the real time factor.
        y = np.arange(len(model_tags))
        width = 0.6 / num_benchmarks
        multiplier = 0
        if num_benchmarks > 1:
            multiplier = -num_benchmarks // 2

        # Create the figure and axis.
        fig, ax = plt.subplots(figsize=(5, 8))
        
        # Plot the real time factors.
        for benchmark, rt_factor in real_time_factors.items():
            offset = width * multiplier
            ax.barh(y + offset, rt_factor, width, label=benchmark)
            multiplier += 1

        # Set the x-axis labels based on max CPU time.
        all_xticks = [1e-2, 1e-1, 1, 10, 100, 1000, 10000, 100000]
        all_xticklabels = ['0.01', '0.1', '1', '10', '100', '1000', '10000', '100000']
        max_value = 0
        for key, value in real_time_factors.items():
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
        plt.xlabel(f'Real Time Factor')
        plt.grid(axis='x', linestyle='--')
        plt.grid(axis='x', which='minor', alpha=0.7, linestyle='--', linewidth=0.4)
        ax.set_axisbelow(True)
        plt.title(self.benchmark_name)
        plt.legend(loc='lower right', fontsize=6)

        # Save the plot.
        plt.tight_layout()
        plt.savefig(target[0], dpi=600)
        plt.close()


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
        for json_path in json_paths:
            with open(json_path) as f:
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
        for json_path in json_paths:
            with open(json_path) as f:
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
                if 'failed' in data:
                    # Create an empty file to satisfy the doit task.
                    with open(html_path, 'w') as f:
                        f.write('')
                else:
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
        

class TaskGenerateDifferentialFlameGraph(StudyTask):
    REGISTRY = []
    def __init__(self, study, tag, model_tuples, event, exe_args=None):
        super(TaskGenerateDifferentialFlameGraph, self).__init__(study)
        self.name = f'generate_differential_flamegraph_{tag}'
        self.tag = tag
        self.study = study
        self.negated = False
        self.negated_flag = ''

        if len(model_tuples) != 2:
            raise ValueError('TaskGenerateDifferentialFlameGraph requires exactly '
                             'two models.')

        self.models = list()
        self.model_names = list()
        self.model_labels = list()
        for model_tuple in model_tuples:
            name = model_tuple[0]
            flags = model_tuple[1]
            model = self.study.get_model(name)
            if model is None:
                print(f' --> {self.name}: Model {name} not found.')
            else:
                self.models.append(model)
                model_name, model_tag = ModelGenerator.get_name_and_tag(
                    model.name, flags)
                self.model_names.append(model_name)
                self.model_labels.append(f'{model.label}\n{model_tag}')
        self.exe_args = exe_args
        self.subdir = get_sub_directory(self.exe_args)
        
        self.results_path = self.study.config['results_path']
        self.data_paths = list()
        for model, model_name in zip(self.models, self.model_names):
            print(f' --> {self.name}: model {model.name} with event {event}')
            self.data_paths.append(
                    os.path.join(self.results_path, model.name, 'perf_realize',
                                 self.subdir, event, f'{model_name}.data'))
            
        self.figures_path = self.study.config['figures_path']
        if not os.path.exists(self.figures_path):
            os.makedirs(self.figures_path)
        
        self.flamegraph_path = os.path.join(self.figures_path, 
                f'differential_flamegraph_{tag}.html')
        self.add_action(self.data_paths, 
                        [self.flamegraph_path], 
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

        print(first)
        print(second)

        command = f'perf script -i {first} > out.stacks1 && '
        command += f'perf script -i {second} > out.stacks2 && '
        command += f'{stackcollapse} out.stacks1 > out.folded1 && '
        command += f'{stackcollapse} out.stacks2 > out.folded2 && '
        command += f'{difffolded} -n out.folded1 out.folded2 | '
        command += f'{flamegraph} {self.negated_flag} > {target[0]} && '
        command += f'rm out.stacks1 out.stacks2 out.folded1 out.folded2'
        try:
            p = subprocess.run(command, check=True, shell=True,
                                cwd=self.study.config['figures_path'])
            if p.returncode != 0:
                raise Exception('Non-zero exit status: code %s.' % p.returncode)
        except Exception as e:
            with open(target[0], 'w') as f:
                f.write('')
            print(f'Error generating differential flamegraph: {e}')