import opensim as osim
import numpy as np

model = osim.Model('subject_walk_scaled.osim')
model.initSystem()

muscles = model.getMuscles()
muscle = osim.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(0))

fl = muscle.get_ActiveForceLengthCurve()

normFiberLengths = np.linspace(0.2, 1.8, 500)

for normFiberLength in normFiberLengths:
    x = osim.Vector(1, normFiberLength)
    multiplier = fl.calcValue(x)
    print(f'Norm fiber length: {normFiberLength}, Multiplier: {multiplier}')
