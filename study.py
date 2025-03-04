import os
import yaml

class Benchmark(object):
    def __init__(self, model, name, metadata=None):
        self.model = model
        self.study = model.study
        self.name = name
        self.metadata = metadata
        self.rel_path = os.path.join(self.model.rel_path, name)
        self.results_exp_path = os.path.join(self.study.config['results_path'],
             self.rel_path)
        self.tasks = list()

    def add_task(self, cls, *args, **kwargs):
        """Add a BenchmarkTask for this benchmark.
        """
        task = cls(self, *args, **kwargs)
        self.tasks.append(task)
        return task


class Model(object):
    def __init__(self, study, filename):
        self.study = study
        self.filename = filename
        self.path = os.path.join(self.study.config['models_path'], self.filename)
        self.name = filename.replace('.osim', '')
        self.rel_path = self.name
        self.results_path = os.path.join(self.study.config['results_path'], 
                                         self.rel_path)
        self.benchmarks = list()
        self.tasks = list()

    def add_benchmark(self, *args, **kwargs):
        """Example: `model.add_benchmark('realize')`"""
        benchmark = Benchmark(self, *args, **kwargs)
        assert not self.contains_benchmark(benchmark.name)
        self.benchmarks.append(benchmark)
        return benchmark
    
    def get_benchmark(self, name):
        for benchmark in self.benchmarks:
            if benchmark.name == benchmark:
                return benchmark
        return None

    def contains_benchmark(self, name):
        return (self.get_benchmark(name) != None)
    
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