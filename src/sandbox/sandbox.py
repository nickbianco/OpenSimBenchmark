import mujoco
import mujoco.viewer as viewer
import numpy as np
import time

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
    <inertial pos="1.0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>
    """

    # Add links iteratively
    for i in range(1, n_links):
        xml += f"""
    <body pos="1.0 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 1.0 0 0"/>
        <inertial pos="1.0 0 0" mass="1.0" diaginertia="1.0 1.0 1.0"/>"""

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
    

model = create_n_link_pendulum(2)
data = mujoco.MjData(model)
model.opt.timestep = 0.001
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
total_steps = 1000

# Benchmark 3: Forward simulation for `sim_time` seconds
reset_data(data)
# mujoco.mj_forward(model, data)
# mujoco.mj_energyPos(model, data)    
# mujoco.mj_energyVel(model, data)
# initial_energy = data.energy[0] + data.energy[1]
# print(f"Initial energy: {initial_energy:.6f} J")

viewer.launch(model, data)

# # Create a viewer
# with viewer.launch_passive(model, data) as v:
#     step = 0
#     while v.is_running() and step < total_steps:
#        for _ in range(total_steps):
#             take_rk4_step(model, data) # Step the simulation
#             print('time:', data.time)
#             time.sleep(model.opt.timestep) # Wait for a short time to control the simulation speed
#             v.sync()  
#             step += 1

# reset_data(data)
# for _ in range(total_steps):
#     mujoco.mj_step(model, data)

# mujoco.mj_energyPos(model, data)
# mujoco.mj_energyVel(model, data)
# final_energy = data.energy[0] + data.energy[1]
# print(f"Final energy: {final_energy:.6f} J")

# mujoco.mj_forward(model, data)
# mujoco.mj_energyPos(model, data)
# mujoco.mj_energyVel(model, data)
# final_energy = data.energy[0] + data.energy[1]
# print(f"Final energy: {final_energy:.6f} J")
        

# mujoco.mj_energyPos(model, data)
# mujoco.mj_energyVel(model, data)
# final_energy = data.energy[0] + data.energy[1]
# energy_conservation = final_energy - initial_energy
# print(f"Initial energy: {initial_energy:.6f} J")
# print(f"Final energy: {final_energy:.6f} J")
# print(f"Energy conservation: {energy_conservation:.6f} J")
# print(f'potential energy: {data.energy[0]}; kinetic energy: {data.energy[1]}')
