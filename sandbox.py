import mujoco
import mujoco.viewer

from time import time
import json
import os

def load_model(model_path):
    model_dir = os.path.dirname(model_path)
    model_filename = os.path.basename(model_path)

    old_dir = os.getcwd()  # Save current directory
    try:
        os.chdir(model_dir)  # Change to the model's directory
        model = mujoco.MjModel.from_xml_path(model_filename)
    finally:
        os.chdir(old_dir)  # Restore original working directory

    return model

model_path = os.path.join('mujoco', 'myo_sim', 'leg', 'myolegs.xml')

model = load_model(model_path)
model.opt.timestep = 0.001
final_time = 5.0

# Disable contacts
for i in range(model.ngeom):
    model.geom_contype[i] = 0
    model.geom_conaffinity[i] = 0

# Disable collision detection            
data = mujoco.MjData(model)

forward_integration_times = list()
real_time_factors = list()
num_steps = int(5.0 / model.opt.timestep)
for iteration in range(100):
    t = time()
    for _ in range(num_steps):
        mujoco.mj_step(model, data, )

    forward_integration_time = time() - t
    forward_integration_times.append(forward_integration_time)
    real_time_factors.append(final_time / forward_integration_time)
    

# Calculate average and standard deviation
average_forward_integration_time = sum(forward_integration_times) / len(forward_integration_times)
average_real_time_factor = sum(real_time_factors) / len(real_time_factors)
std_forward_integration_time = (sum((x - average_forward_integration_time) ** 2 for x in forward_integration_times) / len(forward_integration_times)) ** 0.5
std_real_time_factor = (sum((x - average_real_time_factor) ** 2 for x in real_time_factors) / len(real_time_factors)) ** 0.5

print(f'Forward Integration Time (avg ± std): {average_forward_integration_time:.6f} ± {std_forward_integration_time:.6f} seconds')
print(f'Real Time Factor (avg ± std): {average_real_time_factor:.6f} ± {std_real_time_factor:.6f}')


# Create viewer
# mujoco.viewer.launch(model, data)


# with mujoco.viewer.launch_passive(model, data) as viewer:
#     t_start = time()
#     while data.time < final_time:
#         mujoco.mj_step(model, data)
#         viewer.sync()
#     t_end = time()
#     print(f"Simulated in {(t_end - t_start):.3f} seconds")