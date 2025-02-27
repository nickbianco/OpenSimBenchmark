import sys
import os
import yaml
import numpy as np
with open('config.yaml') as f:
    config = yaml.safe_load(f)

DOIT_CONFIG = {
        'verbosity': 2,
        'default_tasks': None,
        }

# Settings for plots.
import matplotlib.pyplot as plt
plt.rc('font', family='Helvetica, Arial, sans-serif', size=8)
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
study.add_task(InstallDependencies)

# Add models to the study.
model = study.add_model('rajagopal', 'RajagopalLaiUhlrich2023.osim')
flags = ['ignore_activation_dynamics', 'ignore_tendon_compliance', 
         'remove_wrap_objects']
model.add_task(GenerateModels, flags)

# colormap = 'plasma'
# cmap = plt.get_cmap(colormap)
# indices = np.linspace(0, 1.0, len(study.subtalar_suffixes)) 
