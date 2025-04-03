import sys
import os
import yaml
with open('config.yaml') as f:
    config = yaml.safe_load(f)

DOIT_CONFIG = {
        'verbosity': 2,
        'default_tasks': None,
        }

# Settings for plots.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rc('font', size=8)
plt.rc('errorbar', capsize=1.5)
plt.rc('lines', markeredgewidth=1)
plt.rc('legend', fontsize=8)

# Import the perf study objects.
from study import Study

# Register tasks for this project.
from tasks import *

# Define the study.
study = Study('opensim_benchmark')

# Add study tasks.
study.add_task(TaskInstallDependencies)

# MyoSuite tests.
study.add_task(TaskBenchmarkMyoSuiteModels, 1.0, 0.001)

# MuJoCo pendulum tests.
integrators = ['Euler', 'RK4']
steps = [0.01, 0.001, 0.0001]
nlinks = [1, 2, 5, 10, 20, 50, 100]
time = 5.0
for step in steps:
    for nlink in nlinks:
        study.add_task(TaskMuJoCoPendulumBenchmarkRK4Custom, nlink, step, time)
        for integrator in integrators:
            study.add_task(TaskMuJoCoPendulumBenchmark, nlink, step, time, integrator)
study.add_task(TaskAggregatePendulumResults, 'MuJoCo', nlinks, steps, time,
               integrators + ['rk4_custom'])

# Simbody pendulum tests.
integrators = ['euler', 'rk4']
steps = [0.01, 0.001, 0.0001]
nlinks = [1, 2, 5, 10, 20, 50, 100]
time = 5.0
for step in steps:
    for nlink in nlinks:
        study.add_task(TaskSimbodyPendulumBenchmarkRK4Custom, nlink, step, time)
        for integrator in integrators:
            study.add_task(TaskSimbodyPendulumBenchmark, nlink, step, time, integrator)
study.add_task(TaskAggregatePendulumResults, 'Simbody', nlinks, steps, time,
               integrators + ['rk4_custom'])

study.add_task(TaskPlotSimbodyVsMuJoCoSpeedAccuracyTradeoff, nlinks, steps, time, 
               'Euler')
study.add_task(TaskPlotSimbodyVsMuJoCoSpeedAccuracyTradeoff, nlinks, steps, time, 
               'RK4')
study.add_task(TaskPlotSimbodyVsMuJoCoSpeedAccuracyTradeoff, nlinks, steps, time, 
               'rk4_custom')

result_types = ['acceleration_compute_time', 'single_step_time', 
                'forward_integration_time', 'real_time_factor',
                'energy_conservation']
for result_type in result_types:
    for step in steps:
        for integrator in integrators:
            study.add_task(TaskPlotPendulumComparison, ['Simbody', 'MuJoCo'],
                    result_type, nlinks, step, time, integrator)
            
for step in steps:
        study.add_task(TaskPlotPendulumComparison, ['Simbody', 'MuJoCo'],
                    result_type, nlinks, step, time, 'rk4_custom')


# Create Rajagopal models.
study.add_task(TaskCreateRajagopalModels)

# Add models to the study.
def add_model(model_file, label, flags=[]):

    if not os.path.exists(os.path.join(config['models_path'], f'{model_file}.osim')):
        return

    # Add the model to the study.
    model = study.add_model(model_file, label)
    model.add_task(TaskGenerateModels, flags)
    step = 0.001

    # Benchmark tests.
    benchmark_forward_euler = model.add_benchmark('benchmark_forward_euler')
    benchmark_forward_euler.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 1.0, 'step': step})
    benchmark_forward_euler.add_task(TaskPlotBenchmark, benchmark_forward_euler.tasks[-1])

    benchmark_forward_euler.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 5.0})
    benchmark_forward_euler.add_task(TaskPlotBenchmark, benchmark_forward_euler.tasks[-1])

    benchmark_forward_rk4 = model.add_benchmark('benchmark_forward_rk4')
    benchmark_forward_rk4.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 1.0, 'step': step})
    benchmark_forward_rk4.add_task(TaskPlotBenchmark, benchmark_forward_rk4.tasks[-1])

    benchmark_forward_rk4.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 5.0})
    benchmark_forward_rk4.add_task(TaskPlotBenchmark, benchmark_forward_rk4.tasks[-1])

    benchmark_manager_euler = model.add_benchmark('benchmark_manager_euler')
    benchmark_manager_euler.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 1.0, 'step': step})
    benchmark_manager_euler.add_task(TaskPlotBenchmark, benchmark_manager_euler.tasks[-1])

    benchmark_manager_euler.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 5.0})
    benchmark_manager_euler.add_task(TaskPlotBenchmark, benchmark_manager_euler.tasks[-1])

    benchmark_manager_rk4 = model.add_benchmark('benchmark_manager_rk4')
    benchmark_manager_rk4.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 1.0, 'step': step})
    benchmark_manager_rk4.add_task(TaskPlotBenchmark, benchmark_manager_rk4.tasks[-1])

    benchmark_manager_rk4.add_task(TaskRunBenchmark, model.tasks[-1], 
            exe_args={'time': 5.0})
    benchmark_manager_rk4.add_task(TaskPlotBenchmark, benchmark_manager_rk4.tasks[-1])


add_model('Rajagopal', 'Rajagopal', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles'])

add_model('RajagopalPathActuators', 'Rajagopal\npath actuators')

add_model('RajagopalFunctionBasedPaths', 'Rajagopal\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathsNoConstraints', 
          'Rajagopal\nfunction based paths\nno constraints', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathActuators', 
          'Rajagopal\npath actuators\nfunction based paths')

add_model('RajagopalFunctionBasedPathActuatorsNoConstraints',
          'Rajagopal\npath actuators\nfunction based paths\nno constraints',)
          
add_model('RajagopalDGF', 'Rajagopal\nDeGroote-Fregly muscles', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathsDGF', 
          'Rajagopal\nDeGroote-Fregly muscles\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathsDGFNoConstraints', 
          'Rajagopal\nDeGroote-Fregly muscles\nfunction based paths\nno constraints', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'remove_muscles']) 


model_tuples = []
empty_flags = ['']
labels = list()
flags = ['ignore_activation_dynamics',
         'ignore_passive_fiber_force']
model_tuples.append(('Rajagopal', empty_flags))
labels.append('base model')
model_tuples.append(('RajagopalFunctionBasedPaths', empty_flags))
labels.append('function-based paths')
model_tuples.append(('RajagopalFunctionBasedPaths', flags))
labels.append('function-based paths\nno muscle dynamics')
model_tuples.append(('RajagopalFunctionBasedPathsDGF', empty_flags))
labels.append('function-based paths\nsmooth muscles')
model_tuples.append(('RajagopalFunctionBasedPathsDGF', flags))
labels.append('function-based paths\nsmooth muscles\nno muscle dynamics')
model_tuples.append(('Rajagopal', ['remove_muscles']))
labels.append('no muscles')
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_forward_rk4', 
               model_tuples, labels, 'real_time_factor', 1.0, step=0.001)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_forward_rk4', 
               model_tuples, labels, 'real_time_factor', 5.0)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_forward_euler', 
               model_tuples, labels, 'real_time_factor', 1.0, step=0.001)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_forward_euler', 
               model_tuples, labels, 'real_time_factor', 5.0)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_manager_rk4', 
               model_tuples, labels, 'real_time_factor', 1.0, step=0.001)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_manager_rk4', 
               model_tuples, labels, 'real_time_factor', 5.0)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_manager_euler', 
               model_tuples, labels, 'real_time_factor', 1.0, step=0.001)
study.add_task(TaskPlotBenchmarkComparison, 'benchmark_manager_euler', 
               model_tuples, labels, 'real_time_factor', 5.0)
