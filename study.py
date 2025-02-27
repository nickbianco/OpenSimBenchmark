import os
import yaml

class Model(object):
    def __init__(self, study, name, filename):
        self.study = study
        self.name = name
        self.filename = filename
        self.path = os.path.join(self.study.config['models_path'], self.filename)
        # self.conditions = list()
        self.tasks = list()

    # def add_condition(self, *args, **kwargs):
    #     """Example: `cond.add_condition('loaded')`"""
    #     cond = Condition(self, None, *args, **kwargs)
    #     assert not self.contains_condition(cond.name)
    #     self.conditions.append(cond)
    #     return cond
    
    # def get_condition(self, name):
    #     for cond in self.conditions:
    #         if cond.name == name:
    #             return cond
    #     return None

    # def contains_condition(self, name):
    #     return (self.get_condition(name) != None)
    
    def add_task(self, cls, *args, **kwargs):
        """Add a ModelTask for this model.
        """
        task = cls(self, *args, **kwargs)
        self.tasks.append(task)
        return task


class Study(object):
    """
    
    Configuration file
    ------------------
    We expect that the current directory contains a `config.yaml` file with
    the following fields:
    
      - results_path
      - models_path

    An example `config.yaml` may look like the following:

    ```
    results_path: /home/fred/results
    models_path: /home/fred/models
    ```

    The paths do not need to be absolute; they could also be relative. It is
    expected that the `config.yaml` file is not committed to any source code
    repository, since different users might choose different values for these
    settings.
    """
    def __init__(self, name):
        self.name = name

        try:
            with open('config.yaml', 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            raise Exception(str(e) +
                    "\nMake sure there is a config.yaml next to dodo.py")
            
        if not 'results_path' in self.config:
            self.config['results_path'] = '../results'

        self.models = list()
        self.tasks = list()

    def add_model(self, *args, **kwargs):
        model = Model(self, *args, **kwargs)
        assert not self.contains_model(model.name)
        self.models.append(model)
        return model
    
    def get_model(self, name):
        for model in self.models:
            if model.name == name:
                return model
        return None
    
    def contains_model(self, name):
        return (self.get_model(name) != None)

    def add_task(self, cls, *args, **kwargs):
        """Add a StudyTask for this model.
        """
        task = cls(self, *args, **kwargs)
        self.tasks.append(task)
        return task