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


forward = model.add_benchmark('forward')

forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.01})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])

forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.1})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])

forward.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 1.0})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])


forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.01, 'step': 1e-3})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])

forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.1, 'step': 1e-3})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])

forward.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 1.0, 'step': 1e-3})
forward.add_task(TaskPlotBenchmark, forward.tasks[-1])



manager = model.add_benchmark('manager')

manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.01})
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])

manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 0.1})
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])

manager.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'time': 1.0})
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])


manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.01, 'step': 1e-3})
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])

manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 0.1, 'step': 1e-3} )
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])

manager.add_task(TaskRunBenchmark, model.tasks[-1], 
                 exe_args={'time': 1.0, 'step': 1e-3} )
manager.add_task(TaskPlotBenchmark, manager.tasks[-1])


realize = model.add_benchmark('realize')

realize.add_task(TaskRunBenchmark, model.tasks[-1])
realize.add_task(TaskPlotBenchmark, realize.tasks[-1])    

realize.add_task(TaskRunBenchmark, model.tasks[-1], exe_args={'randomize': 1})
realize.add_task(TaskPlotBenchmark, realize.tasks[-1]) 






