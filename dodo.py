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

# Add models to the study.
def add_model(model_file, label, flags):

    model = study.add_model(model_file, label)
    model.add_task(TaskGenerateModels, flags)
    times = [0.01, 0.1, 1.0]

    benchmark_forward = model.add_benchmark('benchmark_forward')
    for time in times:
        benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': time})
        benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

    benchmark_manager = model.add_benchmark('benchmark_manager')
    for time in times:
        benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': time})
        benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])    

    perf_forward = model.add_perf('perf_forward')
    for time in times:
        perf_forward.add_task(TaskRunPerf, model.tasks[-1], exe_args={'time': time})
        perf_forward.add_task(TaskGenerateFlameGraph, perf_forward.tasks[-1])
        perf_forward.add_task(TaskPlotPerf, perf_forward.tasks[-2])

    benchmark_realize = model.add_benchmark('benchmark_realize')
    benchmark_realize.add_task(TaskRunBenchmark, model.tasks[-1])
    benchmark_realize.add_task(TaskPlotBenchmark, benchmark_realize.tasks[-1])

    perf_realize = model.add_perf('perf_realize')
    perf_realize.add_task(TaskRunPerf, model.tasks[-1])
    perf_realize.add_task(TaskGenerateFlameGraph, perf_realize.tasks[-1])


add_model('Rajagopal', 'Rajagopal', 
          flags=['ignore_activation_dynamics', 
                 'ignore_tendon_compliance', 
                 'disable_constraints', 
                 'remove_muscles'])

add_model('RajagopalPathActuators', 'Rajagopal\npath actuators',
          flags=['disable_constraints'])

add_model('RajagopalFunctionBasedPaths', 'Rajagopal\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_tendon_compliance', 
                 'disable_constraints', 
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathActuators', 
          'Rajagopal\npath actuators\nfunction based paths',
          flags=['disable_constraints'])

add_model('RajagopalDGF', 'Rajagopal\nDeGroote-Fregly muscles', 
          flags=['ignore_activation_dynamics', 
                 'ignore_tendon_compliance', 
                 'disable_constraints', 
                 'remove_muscles'])  

add_model('RajagopalFunctionBasedPathsDGF', 
          'Rajagopal\nDeGroote-Fregly muscles\nfunction based paths', 
          flags=['ignore_activation_dynamics', 
                 'ignore_tendon_compliance', 
                 'disable_constraints', 
                 'remove_muscles'])  

# Benchmark 'realize' plots.
# --------------------------
benchmark = 'benchmark_realize'
model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_realize', 
               model_names, benchmark)

model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
model_suffix = '_noconstraints'
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_realize_noconstraints', 
               model_names, benchmark, model_suffix)

model_names = ['Rajagopal', 'RajagopalFunctionBasedPaths']
model_suffix = '_noactdyn_notendyn'
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_realize_noactdyn_notendyn', 
               model_names, benchmark, model_suffix)

# Benchmark 'forward' plots.
# --------------------------
benchmark = 'benchmark_forward'
model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
model_suffix = ''
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_time0.1', 
               model_names, benchmark, model_suffix=model_suffix, exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_time1.0', 
               model_names, benchmark, model_suffix=model_suffix, exe_args={'time': 1.0})

model_names = ['Rajagopal', 'RajagopalPathActuators', 'RajagopalFunctionBasedPaths', 
               'RajagopalFunctionBasedPathActuators']
model_suffix = '_noconstraints'
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_noconstraints_time0.1', 
               model_names, benchmark, model_suffix, exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison, 'Rajagopal_forward_noconstraints_time1.0',
               model_names, benchmark, model_suffix, exe_args={'time': 1.0})

model_names = ['Rajagopal', 'RajagopalFunctionBasedPaths']
model_suffix = '_noactdyn_notendyn'
study.add_task(TaskPlotBenchmarkComparison, 
               'Rajagopal_forward_noactdyn_notendyn_time0.1', 
               model_names, benchmark, model_suffix, exe_args={'time': 0.1})
study.add_task(TaskPlotBenchmarkComparison,
               'Rajagopal_forward_noactdyn_notendyn_time1.0', 
               model_names, benchmark, model_suffix, exe_args={'time': 1.0})


