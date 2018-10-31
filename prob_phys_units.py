#!/usr/bin/env python


from __future__ import print_function
from datamining2 import TypeMiner
from constraint_collector import ConstraintCollector
from constraint_solver import ConstraintSolver
from error_checker import ErrorChecker
from symbol_helper import SymbolHelper
from error_rechecker import ErrorRechecker
from constraint_scoper import ConstraintScoper
import click
import os
from distutils import spawn
from subprocess import Popen
import sys
from time import gmtime, strftime
from shutil import copyfile


# SET PROBABILITY THRESHOLD
PROB_THRESH = 0.5


# SET LOCATIONS OF TRAINING AND TYPES DATA FILES
# training_filepath = os.path.join('', './DATA/variable_units_2017_09_200K_.txt')
training_filepath = os.path.join('', './DATA/2017_06_16_var_names_units_all.txt')
types_filepath = os.path.join('', './DATA/types_data.txt')
suffix_filepath = os.path.join('', './DATA/suffix_units_data.txt')


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def _log(msg):
    print (msg)


@click.command()
@click.argument('target_cpp_file')
@click.option('--correction_file', default='', help='file with unit correction')
@click.option('--should_print_one_line_summary', default='True', help='prints a one-line summary of inconsistencies')
@click.option('--print_constraints/--no-print_constraints', default='False', help='prints constaints used during analysis.')
@click.option('--print_variable_types/--no-print_variable_types', default='False', help='For each variable, prints the physical unit type assignment as a probability distribution.')
def main(target_cpp_file, correction_file, should_print_one_line_summary, print_constraints, print_variable_types):
    original_directory = os.getcwd()

    SHOULD_SUPRESS_OUTPUT_FILES = False  # DURING PARALLEL OPERATION
    SHOULD_USE_CONSTRAINT_SCOPING = False
   

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # TEST FOR CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    if not spawn.find_executable('cppcheck'):
        # CPPCHECK NOT GLOBALLY INSTALLED, CHECK BIN DIRECTORY
        if not os.path.exists('bin/cppcheck'):
            eprint( 'Could not find required program Cppcheck') #todo
            eprint( 'two options: ')
            eprint( '  1.  sudo apt-get install cppcheck')  #todo
            eprint( '  2.  brew install cppcheck')  #todo
            sys.exit(1)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
    # RUN CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    # EXTRACT DIR

    if not os.path.exists(target_cpp_file):
        eprint( 'file does not exist: %s' % target_cpp_file)
        sys.exit(1)

    eprint( 'Processing file %s' % target_cpp_file)
    eprint( 'Attempting to run cppcheck...')
    target_cpp_file_dir = os.path.dirname(target_cpp_file)
    eprint( 'Changing directory to %s' % target_cpp_file_dir)
    os.chdir(target_cpp_file_dir)

    target_cpp_file_base_name = os.path.basename(target_cpp_file)
    dump_filename = os.path.basename(target_cpp_file) + '.dump'

    if not os.path.exists(dump_filename):
        args = ['cppcheck', '--dump', '-I ../include', target_cpp_file_base_name]
        # CREATE CPPCHECK FILE
	made_cfg_dir = False
        if not os.path.exists('cfg'):
            os.makedirs('cfg')
	    made_cfg_dir = True
	copyfile(os.path.join(original_directory, os.path.join('DATA', 'std.cfg')), os.path.join(os.path.join(target_cpp_file_dir, 'cfg'), 'std.cfg'))
        cppcheck_process = Popen(' '.join(args),  shell=True)
        cppcheck_process.communicate()
        if cppcheck_process.returncode != 0:
            eprint( 'cppcheck appears to have failed..exiting with return code %d' % cppcheck_process.returncode)
            sys.exit(1)
        eprint( "Created cppcheck 'dump' file %s" % dump_filename)
        # CLEAN UP CREATE OF CPPCHECK FILE
        if os.path.exists('cfg/std.cfg'):
            try:
                os.remove('cfg/std.cfg')
		if made_cfg_dir:
		    os.rmdir('cfg')
            except:
                # eprint('problem removing cfg folder')
                pass # todo - fail silently for now

    # RETURN TO HOME
    os.chdir(original_directory)

    dump_file = os.path.join(os.path.dirname(target_cpp_file), dump_filename)
    source_file = dump_file.replace('.dump','')


    if correction_file:
        rechecker = ErrorRechecker()
        rechecker.recheck_unit_errors(correction_file, dump_file, source_file)
        return    


    # DO THE MINING
    my_type_miner = TypeMiner(training_filepath, types_filepath, suffix_filepath)
    my_type_miner.train(True)  # True = TRY TO REUSE PREVIOUS TRAINING

    con_collector = ConstraintCollector(my_type_miner)
    con_collector.SHOULD_PRINT_CONSTRAINTS = print_constraints
    con_scoper = ConstraintScoper()
    con_solver = ConstraintSolver(con_collector, con_scoper, SHOULD_USE_CONSTRAINT_SCOPING)
    con_solver.SHOULD_PRINT_VARIABLE_TYPES = print_variable_types
    
    _log("Collecting Constraints ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
    # COLLECT CONSTRAINTS    
    con_collector.main_run_collect(dump_file, source_file)

    _log("Solving Constraints 1 ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
    # SOLVE CONSTRAINTS    
    var2unitproba = con_solver.solve()

    _log("Solving Constraints 2 ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
    # REPEAT
    con_collector.repeat_run_collect(2)
    var2unitproba = con_solver.solve()

    _log("Solving Constraints 3 ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
    # REPEAT
    con_collector.repeat_run_collect(3)
    var2unitproba = con_solver.solve()

    # REPEAT
    _log("Solving Constraints 4 ... %s " % strftime("%Y-%m-%d %H:%M:%S", gmtime()))
    con_collector.repeat_run_collect(4)
    var2unitproba = con_solver.solve()

    # APPLY NEW UNITS
    con_collector.repeat_run_propagate(PROB_THRESH)

    # PRINT VARIABLE-UNITS LIST TO FILE
    if not SHOULD_SUPRESS_OUTPUT_FILES:
        print_variable_units(con_collector.configurations[0], var2unitproba)

    # COLLECT ERRORS
    err_checker = ErrorChecker(dump_file, source_file)
    err_checker.current_file_under_analysis = target_cpp_file    
    err_checker.check_unit_errors(con_collector.configurations[0], con_collector.all_sorted_analysis_unit_dicts[0])

    # PRINT ERRORS TO FILE
    if not SHOULD_SUPRESS_OUTPUT_FILES:
        err_checker.print_unit_errors('errors.txt')
        err_checker.print_var_units_to_check('variable_units_to_check.txt')

        rechecker = ErrorRechecker()
        rechecker.store_state(con_collector.configurations[0], 
                              err_checker.all_errors, 
                              err_checker.variable_units_to_check_as_list)
    
    err_checker.print_one_line_summary()

    if SHOULD_USE_CONSTRAINT_SCOPING:
         compute_results_for_constraint_scopes(target_cpp_file, dump_file, source_file, 
                                               con_collector, con_solver, con_scoper)
    

def print_variable_units(a_cppcheck_configuration, var2unitproba):
    my_symbol_helper = SymbolHelper()
    var_dict = {}
    with open('variables.txt', 'w') as f:
        for t in a_cppcheck_configuration.tokenlist:
            if t.variable:
                if t.is_unit_propagation_based_on_unknown_variable:
                    #continue
                    pass

                #print t.str + ': ' + str(t.units)
                (t, name) = my_symbol_helper.find_compound_variable_and_name_for_variable_token(t)
                if not t:
                    continue
                #eprint("%s, %s, %s" % (name, t.str, t.varId))

                if not my_symbol_helper.should_have_unit(t, name):
                    continue
                    
                if (t.variable.Id, name) not in var_dict:
                    var_dict[(t.variable.Id, name)] = []

                for unit in t.units:
                    if unit not in var_dict[(t.variable.Id, name)]:
                        var_dict[(t.variable.Id, name)].append(unit)

                if (t.variable, name) in var2unitproba:
                    if len(var2unitproba[(t.variable, name)]) >= 2:
                        unit, proba = var2unitproba[(t.variable, name)][0]
                        unit2, proba2 = var2unitproba[(t.variable, name)][1]
                    else:
                        unit, proba = var2unitproba[(t.variable, name)][0]
                        proba2 = 0.0
                    proba = round(proba, 7)
                    proba2 = round(proba2, 7)
                    if (proba > 0.5) and (proba != proba2): #and (unit not in var_dict[(t.variable.Id, name)]):
                        #var_dict[(t.variable.Id, name)].append(unit)
                        var_dict[(t.variable.Id, name)] = [unit]
            
        for (var_id, var_name) in var_dict: 
            var_units = var_dict[(var_id, var_name)]
            f.write("%s, %s, %s\n" % (var_id, var_name, var_units))
    

def compute_results_for_constraint_scopes(target_cpp_file, dump_file, source_file, 
                                          con_collector, con_solver, con_scoper):
    con_scoper.check_constraint_scopes()

    for scope in con_scoper.constraint_scope_list:
        con_scoper.set_current_scope(scope)

        var2unitproba = con_solver.solve()
           
        # REPEAT
        con_collector.repeat_run_collect(5)
        var2unitproba = con_solver.solve()

        # APPLY NEW UNITS
        con_collector.repeat_run_propagate(PROB_THRESH)

        # COLLECT ERRORS
        err_checker = ErrorChecker(dump_file, source_file)
        err_checker.current_file_under_analysis = target_cpp_file    
        err_checker.check_unit_errors(con_collector.configurations[0], con_collector.all_sorted_analysis_unit_dicts[0])

        err_checker.print_one_line_summary()    


if __name__ == "__main__":
    main()

