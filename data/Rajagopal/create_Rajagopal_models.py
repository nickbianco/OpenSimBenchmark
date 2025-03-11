import os
import opensim as osim
import shutil

models_dir = os.path.join('..', '..', 'models')

# Copy 'subject_walk_scaled.osim' to the models/Rajagopal.osim
dst = os.path.join(models_dir, 'Rajagopal.osim')
shutil.copyfile('subject_walk_scaled.osim', dst)

# PathActuator model.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplaceMusclesWithPathActuators())
model = modelProcessor.process()
model.finalizeConnections()
model.initSystem()
model.printToXML(os.path.join(models_dir, 'RajagopalPathActuators.osim'))

# Function-based path model.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
    'subject_walk_scaled_FunctionBasedPathSet.xml'))
model = modelProcessor.process()
model.finalizeConnections()
model.initSystem()
model.printToXML(os.path.join(models_dir, 'RajagopalFunctionBasedPaths.osim'))

# Function-based PathActuators model.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplaceMusclesWithPathActuators())
modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
    'subject_walk_scaled_FunctionBasedPathSet.xml'))
model = modelProcessor.process()
model.finalizeConnections()
model.initSystem()
model.printToXML(os.path.join(models_dir, 'RajagopalFunctionBasedPathActuators.osim'))

# DeGrooteFregly2016Muscle model.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
model = modelProcessor.process()
model.finalizeConnections()
model.initSystem()
model.printToXML(os.path.join(models_dir, 'RajagopalDGF.osim'))

# Function-based DeGrooteFregly2016Muscle model.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
    'subject_walk_scaled_FunctionBasedPathSet.xml'))
model = modelProcessor.process()
model.finalizeConnections()
model.initSystem()
model.printToXML(os.path.join(models_dir, 'RajagopalFunctionBasedPathsDGF.osim'))
