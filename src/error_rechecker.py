from unit_error_types import UnitErrorTypes
from error_checker import ErrorChecker
from tree_walker import TreeWalker
import cps_constraints as con
import cppcheckdata
import pickle
import os
from operator import itemgetter



class ErrorRechecker:
    ''' IMPLEMENTATION OF USER-ASSISTED ERROR RECHECKING
    '''

    def __init__(self):
        self.cppcheck_pkl_filename = 'cppcheck_config.pkl'
        self.errors_pkl_filename = 'error_list.pkl'
        self.varlist_pkl_filename = 'var_units_to_check_list.pkl'


    def store_state(self, a_cppcheck_configuration, errors, variable_units_to_check_as_list):
        tokenlist = {}
        for t in a_cppcheck_configuration.tokenlist:
            tokenlist[t.Id] = (t.units, t.isKnown, t.is_unit_propagation_based_on_constants,
                               t.is_unit_propagation_based_on_unknown_variable, 
                               t.is_unit_propagation_based_on_weak_inference, t.isRoot, t.isDimensionless)

        pickle.dump(tokenlist, open(self.cppcheck_pkl_filename, 'wb'), pickle.HIGHEST_PROTOCOL)

        for e in errors:
            e.token = e.token.Id
            if e.token_left:
                e.token_left = e.token_left.Id
            if e.token_right:
                e.token_right = e.token_right.Id
    
        pickle.dump(errors, open(self.errors_pkl_filename, 'wb'), pickle.HIGHEST_PROTOCOL)

        varlist = {}
        for (isKnown, rank, var, var_name, units, linenrs) in variable_units_to_check_as_list:
            varlist[(var.Id, var_name)] = rank

        pickle.dump(varlist, open(self.varlist_pkl_filename, 'wb'), pickle.HIGHEST_PROTOCOL)


    def get_cppcheck_config_data_structure(self, dump_file):
        data = cppcheckdata.parsedump(dump_file)
        for c in data.configurations[:1]:
            return c


    def load_state(self, a_cppcheck_configuration):
        tokenlist = {}
        errors = []

        if not (os.path.exists(self.cppcheck_pkl_filename) and \
                os.path.exists(self.errors_pkl_filename) and \
                os.path.exists(self.varlist_pkl_filename)):
            return

        tokenlist = pickle.load(open(self.cppcheck_pkl_filename, 'rb'))

        for t in a_cppcheck_configuration.tokenlist:
            (units, isKnown, is_unit_propagation_based_on_constants,
             is_unit_propagation_based_on_unknown_variable,
             is_unit_propagation_based_on_weak_inference, isRoot, isDimensionless) = tokenlist[t.Id]
        
            t.units = units
            t.isKnown = isKnown
            t.is_unit_propagation_based_on_constants = is_unit_propagation_based_on_constants
            t.is_unit_propagation_based_on_unknown_variable = is_unit_propagation_based_on_unknown_variable
            t.is_unit_propagation_based_on_weak_inference = is_unit_propagation_based_on_weak_inference
            t.isRoot = isRoot
            t.hasVarOperand = False
            t.isDimensionless = isDimensionless

        errors = pickle.load(open(self.errors_pkl_filename, 'rb'))

        for t in a_cppcheck_configuration.tokenlist:
            for e in errors:
                if e.token == t.Id:
                    e.token = t
                if e.token_left and e.token_left == t.Id:
                    e.token_left = t
                if e.token_right and e.token_right == t.Id:
                    e.token_right = t    

        varlist = pickle.load(open(self.varlist_pkl_filename, 'rb'))

        return (errors, varlist)


    def apply_and_propagate_units(self, tw, root_token):
        break_point = 1000
        i=0

        # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
        tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)
    
        tw.generic_recurse_and_apply_function(root_token, tw.apply_correction_units)
            
        # CONTINUE TO ATTEMPT CHANGES UNTIL CHANGES CEASE
        while tw.was_some_unit_changed:  
            if i>break_point:
                s = "BREAKING WHILE LOOP AT %d" % break_point
                raise ValueError(s)
                return
            i+=1
            tw.was_some_unit_changed = False
            # LOOK FOR EARLY ABANDONMENT OF THIS AST
            if not tw.found_units_in_this_tree:
                break
            ### PROPAGATE UNITS
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_dot_connectors)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_double_colon)                
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_square_brackets)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_assignment)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_abs_fabs_floor_ceil)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_fmod_fmodf_fmodl)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
            # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
            # tw.generic_recurse_and_apply_function(root_token, tw.collect_function_param_units_and_decorate_function)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
        # END -- WHILE LOOP    
 

    def recheck_unit_errors(self, correction_file, dump_file, source_file):
        with open(correction_file) as f:
            for var_result in (line.rstrip('\n') for line in f):
                var_name, var_unit = var_result.split(',', 1)
                var_name, var_unit = var_name.strip(), var_unit.strip()
                var_unit = eval(var_unit) 
                con.phys_corrections[var_name] = var_unit

        #print "phys_corrections: %s" % con.phys_corrections

        a_cppcheck_configuration = self.get_cppcheck_config_data_structure(dump_file)
        errors, varlist = self.load_state(a_cppcheck_configuration)

        err_checker = ErrorChecker(dump_file, source_file)
        show_high_confidence=True 
        show_low_confidence=False

        for e in errors:
            is_high_confidence = not e.is_warning
            is_low_confidence = e.is_warning

            if is_high_confidence and not show_high_confidence:
                continue
            if is_low_confidence and not show_low_confidence:
                continue

            if e.ERROR_TYPE == UnitErrorTypes.VARIABLE_MULTIPLE_UNITS:
                tw = TreeWalker(None)
                self.apply_and_propagate_units(tw, e.token)

                # TRACK VARIABLE WITH MULTIPLE UNITS
                if len(e.token.astOperand2.units) > 1:
                    e.units_when_multiple_happened = e.token.astOperand2.units
                    err_checker.all_errors.append(e)

                if e.token_left.isKnown:
                    # TRACK VARIABLE WITH MULTIPLE UNITS
                    if (len(e.token.astOperand2.units) == 1) and (e.token_left.units != e.token.astOperand2.units):
                        units = []
                        units.extend(e.token_left.units)
                        units.extend(e.token.astOperand2.units)
                        e.units_when_multiple_happened = units
                        err_checker.all_errors.append(e)

            elif e.ERROR_TYPE == UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS:
                tw = TreeWalker(None)
                self.apply_and_propagate_units(tw, e.token_left)
                self.apply_and_propagate_units(tw, e.token_right)
            
                # UPDATE UNITS AT BOTH CALL POINTS
                e.units_at_first_assignment = e.token_left.units
                e.units_when_multiple_happened = e.token_right.units

                # CHECK UNITS OF FIRST CALL POINT AGAINST THE OTHER CALL POINT
                if e.units_when_multiple_happened != e.units_at_first_assignment:
                    err_checker.all_errors.append(e)

            elif e.ERROR_TYPE == UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS:
                tw = TreeWalker(None)
                self.apply_and_propagate_units(tw, e.token)
                err_checker.have_found_addition_error_on_this_line = False
                tw.generic_recurse_and_apply_function(e.token, err_checker.error_check_addition_of_incompatible_units_recursive)

            elif e.ERROR_TYPE == UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS:
                tw = TreeWalker(None)
                self.apply_and_propagate_units(tw, e.token)
                tw.generic_recurse_and_apply_function(e.token, err_checker.error_check_comparison_recursive)

            else:
                err_checker.all_errors.append(e)

        print "Error_Rechecker:"                 
        err_checker.pretty_print()
        err_checker.print_unit_errors('errors_2.txt')
        self.print_var_units_to_check(err_checker, varlist, 'variable_units_to_check_2.txt')


    def print_var_units_to_check(self, err_checker, varlist, check_file):
        # convert var_units_for_check dictionary to a list ordered by ranking
        for (var, var_name) in err_checker.variable_units_to_check:
            value = err_checker.variable_units_to_check[(var, var_name)]
            isKnown = value[0]
            rank = 1.0
            if (var.Id, var_name) in varlist:
                rank = varlist[(var.Id, var_name)]
            err_checker.variable_units_to_check_as_list.append((isKnown, rank, var, var_name, value[1], value[2]))
        err_checker.variable_units_to_check_as_list = sorted(err_checker.variable_units_to_check_as_list, key=itemgetter(0, 1))
            
        with open(check_file, 'w') as f:
            for (isKnown, rank, var, var_name, units, linenrs) in err_checker.variable_units_to_check_as_list:
                f.write("%s, %s, %s, %s, %s\n" % (linenrs[0], rank, var.Id, var_name, units))
    

