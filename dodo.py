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

# Create Rajagopal models.
study.add_task(TaskCreateRajagopalModels)

# Add models to the study.
def add_model(model_file, label, flags):

    if not os.path.exists(os.path.join(config['models_path'], f'{model_file}.osim')):
        return

    # Add the model to the study.
    model = study.add_model(model_file, label)
    model.add_task(TaskGenerateModels, flags)
    times = [0.01, 0.1, 1.0]

    # Benchmark tests.
    benchmark_forward = model.add_benchmark('benchmark_forward')
    for time in times:
        benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                                   exe_args={'time': time})
        benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

    benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                                exe_args={'time': 0.1, 'step': 0.001})
    benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1]) 

    benchmark_realize = model.add_benchmark('benchmark_realize')
    benchmark_realize.add_task(TaskRunBenchmark, model.tasks[-1])
    benchmark_realize.add_task(TaskPlotBenchmark, benchmark_realize.tasks[-1])

    # Linux 'perf' tests.
    perf_forward = model.add_perf('perf_forward')
    events = ['cycles', 'cache-misses']
    for time in times:
        for event in events:
            perf_forward.add_task(TaskRunPerf, event, model.tasks[-1], 
                                  exe_args={'time': time})
            perf_forward.add_task(TaskGenerateFlameGraph, perf_forward.tasks[-1])
            perf_forward.add_task(TaskPlotPerf, perf_forward.tasks[-2])

    perf_forward.add_task(TaskRunPerf, 'cycles', model.tasks[-1], 
                          exe_args={'time': 0.1, 'step': 0.0001})
    perf_forward.add_task(TaskGenerateFlameGraph, perf_forward.tasks[-1])
    perf_forward.add_task(TaskPlotPerf, perf_forward.tasks[-2])

    perf_realize = model.add_perf('perf_realize')
    for event in events:
        perf_realize.add_task(TaskRunPerf, event, model.tasks[-1])
        perf_realize.add_task(TaskGenerateFlameGraph, perf_realize.tasks[-1])
        perf_realize.add_task(TaskPlotPerf, perf_realize.tasks[-2])


add_model('Rajagopal', 'Rajagopal', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'disable_constraints', 
                 'remove_muscles'])

add_model('RajagopalPathActuators', 'Rajagopal\npath actuators',
          flags=['disable_constraints'])

add_model('RajagopalFunctionBasedPaths', 'Rajagopal\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'disable_constraints', 
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathActuators', 
          'Rajagopal\npath actuators\nfunction based paths',
          flags=['disable_constraints'])

add_model('RajagopalDGF', 'Rajagopal\nDeGroote-Fregly muscles', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'disable_constraints', 
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathsDGF', 
          'Rajagopal\nDeGroote-Fregly muscles\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_passive_fiber_force',
                 'disable_constraints', 
                 'remove_muscles'])  

# Benchmark 'realize' plots.
# --------------------------
benchmark = 'benchmark_realize'
model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators', 'RajagopalDGF',
               'RajagopalFunctionBasedPathsDGF']
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_realize', 
               model_names, benchmark)

model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators', 'RajagopalDGF',
               'RajagopalFunctionBasedPathsDGF']
model_suffix = '_noconstraints'
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_realize_noconstraints', 
               model_names, benchmark, model_suffix)


# Benchmark 'forward' plots.
# --------------------------
benchmark = 'benchmark_forward'
model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
model_suffix = ''
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_time0.1', 
               model_names, benchmark, model_suffix=model_suffix, 
               exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_time1.0', 
               model_names, benchmark, model_suffix=model_suffix, 
               exe_args={'time': 1.0})

model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
model_suffix = '_noconstraints'
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_noconstraints_time0.1', 
               model_names, benchmark, model_suffix, exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_noconstraints_time1.0',
               model_names, benchmark, model_suffix, exe_args={'time': 1.0})


# Pendulum models.
links = [1, 2, 5, 10, 20, 50, 100]
study.add_task(TaskCreatePendulumModels, links)

flags = []
for link in links:
    model_file = f'Pendulum_{link}'
    add_model(f'{link}link_pendulum', f'{link} link pendulum', flags)

model_names = [f'{link}link_pendulum' for link in links]
study.add_task(TaskPlotBenchmarkComparison, 'pendulum_realize', 
               model_names, 'benchmark_realize')

model_suffix = ''
study.add_task(TaskPlotBenchmarkComparison, 'pendulum_forward_time0.1', 
               model_names, 'benchmark_forward', model_suffix=model_suffix,
               exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison, 'pendulum_forward_time1.0', 
               model_names, 'benchmark_forward', model_suffix=model_suffix,
               exe_args={'time': 1.0})


# Frames per second
empty_flags = ['']
model_tuples = []
for link in links:
    model_tuples.append((f'{link}link_pendulum', empty_flags))
study.add_task(TaskPlotFramesPerSecondRealize, 'pendulum', model_tuples)
# study.add_task(TaskPlotFramesPerSecondForwardFixedStep, 'pendulum', model_tuples)

model_tuples = []
empty_flags = ['']
model_tuples.append(('Rajagopal', ['remove_muscles']))
model_tuples.append(('Rajagopal', flags))
model_tuples.append(('RajagopalDGF', flags))
model_tuples.append(('RajagopalFunctionBasedPaths', flags))
model_tuples.append(('RajagopalFunctionBasedPathsDGF', flags))
model_tuples.append(('RajagopalPathActuators', flags))
model_tuples.append(('RajagopalFunctionBasedPathActuators', flags))
study.add_task(TaskPlotFramesPerSecondRealize, 'Rajagopal', model_tuples)
study.add_task(TaskPlotFramesPerSecondForwardFixedStep, 'Rajagopal', model_tuples)


model_tuples = []
empty_flags = ['']
flags = ['ignore_activation_dynamics', 
         'ignore_passive_fiber_force']
model_tuples.append(('Rajagopal', ['remove_muscles']))
model_tuples.append(('Rajagopal', empty_flags))
model_tuples.append(('Rajagopal', flags))
model_tuples.append(('RajagopalDGF', empty_flags))
model_tuples.append(('RajagopalDGF', flags))
model_tuples.append(('RajagopalFunctionBasedPaths', empty_flags))
model_tuples.append(('RajagopalFunctionBasedPaths', flags))
model_tuples.append(('RajagopalFunctionBasedPathsDGF', empty_flags))
model_tuples.append(('RajagopalFunctionBasedPathsDGF', flags))
study.add_task(TaskPlotFramesPerSecondRealize, 
               'Rajagopal_noactdyn_nopassive', model_tuples)
study.add_task(TaskPlotFramesPerSecondForwardFixedStep, 
               'Rajagopal_noactdyn_nopassive', model_tuples)


model_tuples = []
empty_flags = ['']
flags = ['disable_constraints']
model_tuples.append(('Rajagopal', ['remove_muscles']))
model_tuples.append(('Rajagopal', ['remove_muscles', 'disable_constraints']))
model_tuples.append(('Rajagopal', empty_flags))
model_tuples.append(('Rajagopal', flags))
model_tuples.append(('RajagopalDGF', empty_flags))
model_tuples.append(('RajagopalDGF', flags))
model_tuples.append(('RajagopalFunctionBasedPaths', empty_flags))
model_tuples.append(('RajagopalFunctionBasedPaths', flags))
model_tuples.append(('RajagopalFunctionBasedPathsDGF', empty_flags))
model_tuples.append(('RajagopalFunctionBasedPathsDGF', flags))
model_tuples.append(('RajagopalPathActuators', empty_flags))
model_tuples.append(('RajagopalPathActuators', flags))
model_tuples.append(('RajagopalFunctionBasedPathActuators', empty_flags))
model_tuples.append(('RajagopalFunctionBasedPathActuators', flags))
study.add_task(TaskPlotFramesPerSecondRealize, 'Rajagopal_noconstraints', model_tuples)
# study.add_task(TaskPlotFramesPerSecondForwardFixedStep, 'Rajagopal_noconstraints', model_tuples)


