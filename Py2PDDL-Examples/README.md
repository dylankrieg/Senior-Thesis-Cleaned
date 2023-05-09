# Py2PPDL Examples

These examples illustate the pipeline for generating solvable domain.pddl and problem.pddl files that are described in Python using Py2PPDL (https://github.com/remykarem/py2pddl).

- blocks.py demonstrates modeling a PDDL Problem and Domain using statically set classes

- dynamicBlocks.ipynb adds onto this with dynamically generated initial and goal states

# Getting Started:

1. Install Py2PDDL

$ python3 -m pip install git+https://github.com/remykarem/py2pddl#egg=py2pddl

2. Clone fast downward to this directory

$ git clone https://github.com/aibasel/downward

3. Build fast downward

$ cd downward

$ ./build.py release 

4. Generate domain/problem PDDL files

$ python3 blocks.py

5. Solve using fast downward

$ bash runDownward.bash 

6. View planner output

$ cat sas_plan
