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
model = study.add_model('RajagopalLaiUhlrich2023.osim')

flags = ['ignore_activation_dynamics', 'ignore_tendon_compliance', 'remove_wrap_objects', 
         'disable_constraints', 'remove_muscles']
model.add_task(TaskGenerateModels, flags)


benchmark_forward = model.add_benchmark('benchmark_forward')

benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.01})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.1})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 1.0})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])


benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.01, 'step': 1e-3})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.1, 'step': 1e-3})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])

benchmark_forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 1.0, 'step': 1e-3})
benchmark_forward.add_task(TaskPlotBenchmark, benchmark_forward.tasks[-1])



benchmark_manager = model.add_benchmark('benchmark_manager')

benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.01})
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])

benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.1})
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])

benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 1.0})
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])


benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.01, 'step': 1e-3})
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])

benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.1, 'step': 1e-3} )
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])

benchmark_manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 1.0, 'step': 1e-3} )
benchmark_manager.add_task(TaskPlotBenchmark, benchmark_manager.tasks[-1])


benchmark_realize = model.add_benchmark('benchmark_realize')

benchmark_realize.add_task(TaskRunBenchmark, model.tasks[-1])
benchmark_realize.add_task(TaskPlotBenchmark, benchmark_realize.tasks[-1])    

benchmark_realize.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'randomize': 1})
benchmark_realize.add_task(TaskPlotBenchmark, benchmark_realize.tasks[-1]) 


perf_forward = model.add_perf('perf_forward')

perf_forward.add_task(TaskRunPerf, model.tasks[-1], exe_args={'time': 0.001})
perf_forward.add_task(TaskGeneratePerfSVG, perf_forward.tasks[-1])

perf_forward.add_task(TaskRunPerf, model.tasks[-1], exe_args={'time': 0.1})
perf_forward.add_task(TaskGeneratePerfSVG, perf_forward.tasks[-1])


perf_realize = model.add_perf('perf_realize')

perf_realize.add_task(TaskRunPerf, model.tasks[-1])
perf_realize.add_task(TaskGeneratePerfSVG, perf_realize.tasks[-1])






