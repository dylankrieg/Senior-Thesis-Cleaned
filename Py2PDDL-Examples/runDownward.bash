python3 -m py2pddl.parse blocks.py
./downward/fast-downward.py domain.pddl problem.pddl --search "astar(lmcut())"
cat sas_plan
