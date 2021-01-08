

# PHYS - A physical units consistency checker for ROS C++


# [https://unl-nimbus-lab.github.io/phys/](https://unl-nimbus-lab.github.io/phys/)


CITE:
```
@inproceedings{DBLP:conf/sigsoft/KateOZEX18,
  author    = {Sayali Kate and
               John{-}Paul Ore and
               Xiangyu Zhang and
               Sebastian G. Elbaum and
               Zhaogui Xu},
  editor    = {Gary T. Leavens and
               Alessandro Garcia and
               Corina S. Pasareanu},
  title     = {Phys: probabilistic physical unit assignment and inconsistency detection},
  booktitle = {Proceedings of the 2018 {ACM} Joint Meeting on European Software Engineering
               Conference and Symposium on the Foundations of Software Engineering,
               {ESEC/SIGSOFT} {FSE} 2018, Lake Buena Vista, FL, USA, November 04-09,
               2018},
  pages     = {563--573},
  publisher = {{ACM}},
  year      = {2018},
  url       = {https://doi.org/10.1145/3236024.3236035},
  doi       = {10.1145/3236024.3236035},
  timestamp = {Fri, 06 Nov 2020 10:10:15 +0100},
  biburl    = {https://dblp.org/rec/conf/sigsoft/KateOZEX18.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

## FILES

|FILE | PURPOSE |
|-----|---------| 
| prop_phys_units.py  |  Main file that runs phys. |
| constraint_collector.py | |
| constraint_scoper.py | |
| constraint_solver.py | |
| cppcheckdata.py  |  Library to parse CPPCheck dump files, (parsed Code) |
| cps_constraints.py |  |
| datamining.py | not used. |
| datamining2.py | |
| datamining_self_var2type.pkl | storage of priors |
| datamining_self_vars.pkl | storage of priors |
| error_checker.py   | from Phriky, traverses abstract syntax tree to find physical unit inconsistencies. |
| error_rechecker.py | from Phriky, traverses abstract syntax tree to find physical unit inconsistencies. |
| pgm/   | Probablistic graphical models from http://libDAI.org |
| str_utils.py  | helper functions for parsing strings |
| symbol_helper.py  | from Phriky, mapping between ROS attributes of shared libraries and Physical Unit Types (PUTs). |
| tree_walker.py | visitor pattern implementation to decorate the abstract syntax tree with PUTs. |
| unit_error.py | physical unit error container object.  One is generated per unit error. |
| unit_error_types.py | data structure to defind the different types of physical unit errors. |
| var_name_heuristic.py |  |





