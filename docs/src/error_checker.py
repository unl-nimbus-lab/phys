#!/usr/bin/python

from unit_error import UnitError
from unit_error_types import UnitErrorTypes
from tree_walker import TreeWalker
from symbol_helper import SymbolHelper
import cps_constraints as con
import os.path
from operator import itemgetter
import copy
import cppcheckdata


class ErrorChecker:
    ''' IMPLEMENTATION OF MAIN ERROR CHECKING
    '''

    def __init__(self, dump_file, source_file): 
        self.dump_file = dump_file 
        self.current_file_under_analysis = ''
	self.source_file = source_file
        self.source_file_exists = False
        self.source_file_lines = []
        self.prepare_source_file_for_reading()
        self.all_errors = []
        #self.all_warnings = []
        self.symbol_helper = SymbolHelper()
        self.have_found_addition_error_on_this_line = False
        self.marked_as_low_confidence = []
        self.variable_units_to_check = {}
        self.variable_units_to_check_as_list = []

    
    def prepare_source_file_for_reading(self):
        # OPEN SOURCE FILE IF ERRORS FOUND
        self.source_file_exists = os.path.isfile(self.source_file)
        self.source_file_lines = []
        if self.source_file_exists: 
            with open(self.source_file, 'r') as f:
                self.source_file_lines = f.readlines()
        else:
            print 'No source file found at: %s' % self.source_file


    def get_file_URI_where_error_occured(self, e):
        ''' GETS THE BEST URI POSSIBLE OF THE FILE CONTAINING THE ERROR
            input: None
            returns: string representing URI of file with error
            '''
        if not e.token or not self.current_file_under_analysis:
            return ''
        # REMOVE FILENAME FROM END
        base_path = re.sub('(\w|\.)*$', '', self.current_file_under_analysis)
        # APPEND FILE NAME - MIGHT INCLUDE SOMETHING LIKE "../include/laser_transform_core.h"
        return base_path + e.token.file


    def check_unit_errors(self, cppcheck_configuration_unit, sorted_analysis_unit_dict):
        # COLLECT ERRORS
        self.error_check_function_args_consistent(cppcheck_configuration_unit)
        self.error_check_addition_of_incompatible_units(sorted_analysis_unit_dict)
        self.error_check_comparisons(sorted_analysis_unit_dict)
        #self.error_check_logical_operators(sorted_analysis_unit_dict)
        self.error_check_multiple_units()

        # CHECK ERRORS WITH TOP3 UNITS
        self.check_errors_with_low_confidence_when_top3_units(cppcheck_configuration_unit, sorted_analysis_unit_dict)

        # DISPLAY ERRORS
        self.pretty_print()


    def error_check_multiple_units(self):
        ''' MULTIPLE_UNIT_TYPE ASSIGNMENT ERROR CHECKING IMPLEMENTATION 
            returns: none
            side_effects: might add UnitError objects to self.all_errors list
            '''
        for root_token, token, name, units, isKnownRhs in con.multi_unit_variables:
            new_error = UnitError()   
            new_error.ERROR_TYPE = UnitErrorTypes.VARIABLE_MULTIPLE_UNITS
            new_error.linenr = token.linenr
            new_error.token = root_token
            new_error.token_left = token
            new_error.var_name = name
            new_error.units_when_multiple_happened = units
            new_error.dont_check_for_warning = isKnownRhs
            new_error.is_unit_propagation_based_on_constants = \
                    root_token.is_unit_propagation_based_on_constants
            new_error.is_unit_propagation_based_on_unknown_variable = \
                    root_token.is_unit_propagation_based_on_unknown_variable
            if new_error.is_unit_propagation_based_on_constants or \
                    new_error.is_unit_propagation_based_on_unknown_variable:
                new_error.is_warning = True

            if (not new_error.dont_check_for_warning) and (new_error.linenr in self.marked_as_low_confidence):
                new_error.is_warning = True

            self.all_errors.append(new_error)


    def error_check_function_args_consistent(self, cppcheck_configuration_unit):
        ''' VERIFIES UNIT CONSISTENCY OF FUNCTIONS AT EVERY CALL POINT
            input: cppcheck configuration unit from dump
            returns: none
            side_effects: might add UnitError objects to self.all_errors list
            '''
        # FOR EACH FUNCTION
        for f in cppcheck_configuration_unit.functions:
            # FOR EACH ARG IN A FUNCTION
            for arg_list_of_call_points in f.arg_units:
                # FOR EACH TIME THE FUNCTION WAS CALLED
                error_found = False
                new_error = UnitError()
                first_call_point_with_units = None
                for call_point in arg_list_of_call_points:
                    # FIND SOME CALL POINT WITH UNITS
                    if not first_call_point_with_units:
                        if call_point['units']:
                            first_call_point_with_units = call_point
                        continue
                    # CHECK UNITS OF FIRST CALL POINT AGAINST ALL OTHERS
                    if call_point['units'] and (call_point['units'] != first_call_point_with_units['units']):
                        error_found = True
                        # FOUND DIFFERENT UNITS AT TWO DIFFERENT CALL POINTS
                        new_error.var_name = f.name
                        new_error.ERROR_TYPE = UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS
                        new_error.token = first_call_point_with_units['function']
                        # FIRST ASSIGNMENT
                        new_error.set_primary_line_number(first_call_point_with_units['linenr'])
                        new_error.linenr_at_first_unit_assignment = first_call_point_with_units['linenr']
                        new_error.units_at_first_assignment = first_call_point_with_units['units']
                        new_error.token_left = first_call_point_with_units['token']
                        # SECOND (DIFFERENT) ASSIGNMENT
                        new_error.linenr_of_multiple_unit_assignment = call_point['linenr']
                        new_error.units_when_multiple_happened = call_point['units']
                        new_error.token_right = call_point['token']
                        break
                if error_found:
                    # IF NO RETURN UNITS, REPORT AS A WARNING
                    if (f.return_units == []) or f.maybe_generic_function:
                        new_error.is_warning = True
                    # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                    if self.source_file_exists:
                        # TODO: resolve relative link to different file and load source
                        if new_error.linenr_at_first_unit_assignment <= len(self.source_file_lines) and \
                                new_error.linenr_of_multiple_unit_assignment <= len(self.source_file_lines):
                            new_error.source_code_at_first_assignment = \
                                    self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                            new_error.source_code_when_multiple_units_happened = \
                                    self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                    # COLLECT ERROR
                    self.all_errors.append(new_error)      


    def error_check_addition_of_incompatible_units(self, sorted_analysis_unit_dict):
        ''' ERROR CHECK ADDITION OF INCOMPATIBLE UNITS
            input: sorted analysis unit dictionary of functions
            returns: none
            side_effects: might add UnitError objects to self.all_errors list
            '''            
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(None)
            for root_token in function_dict['root_tokens']:
                self.have_found_addition_error_on_this_line = False
                tw.generic_recurse_and_apply_function(root_token, self.error_check_addition_of_incompatible_units_recursive)


    def error_check_addition_of_incompatible_units_recursive(self, token, left_token, right_token):
        ''' ERROR CHECK ADDITION OF INCOMPATIBLE UNITS - RESURSIVE TARGET
            input:  token (cppcheck token object),
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''            
        if token.str in ['+', '-', '+=', '-=']:
            # THIS IS ADDITION OR SUBTRACTION
            if token.astOperand1 and token.astOperand2:
                if token.astOperand1.units and token.astOperand2.units:        
                    # BOTH CHILDREN HAVE UNITS
                    if token.astOperand1.units != token.astOperand2.units:
                        if not self.have_found_addition_error_on_this_line:
                            # UNIT MISMATCH ON ADDITION
                            new_error = UnitError()
                            new_error.ERROR_TYPE = UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS

                            new_error.is_unit_propagation_based_on_constants = token.is_unit_propagation_based_on_constants
                            new_error.is_unit_propagation_based_on_unknown_variable = \
                                    token.is_unit_propagation_based_on_unknown_variable
                            if token.is_unit_propagation_based_on_constants or \
                                    token.is_unit_propagation_based_on_unknown_variable:
                                new_error.is_warning = True

                            if (not new_error.is_warning):
                                new_error.is_warning = self.check_if_error_with_low_confidence(token, left_token, right_token)
                                if new_error.is_warning:
                                    self.marked_as_low_confidence.append(token.linenr)

                            # LINENR 
                            new_error.linenr = token.linenr
                            new_error.token_left = left_token
                            new_error.token_right = right_token
                            new_error.token = token
                            new_error.var_name = self.get_var_name(left_token)
                            # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                            if self.source_file_exists:
                                pass
                            # COLLECT ERROR
                            self.all_errors.append(new_error)
                            self.have_found_addition_error_on_this_line = True


    def error_check_comparisons(self, sorted_analysis_unit_dict):
        ''' ERR CHECK COMPARISION OF UNITS OVER LOGICAL OPERATORS  
            input: sorted analysis unit dictionary of functions
            returns: none
            side_effects: might add UnitError objects to self.all_errors list
            '''            
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(None)
            for root_token in function_dict['root_tokens']:
                tw.generic_recurse_and_apply_function(root_token, self.error_check_comparison_recursive)


    def error_check_comparison_recursive(self, token, left_token, right_token):
        ''' COMPARISON OPERATORS - MUST BE THE SAME ON BOTH SIDES
            input:  token (cppcheck token object),
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''
        if token.isComparisonOp:
            if left_token and left_token.units and right_token and right_token.units:  
                # BOTH HAVE UNITS
                if left_token.units != right_token.units:
                    # UNIT MISMATCH ON COMPARISON
                    new_error = UnitError()
                    new_error.ERROR_TYPE = UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS
                    # LINENR 
                    new_error.linenr = token.linenr
                    new_error.token_left = left_token
                    new_error.token_right = right_token
                    new_error.token = token
                    new_error.var_name = self.get_var_name(left_token)
                    new_error.is_unit_propagation_based_on_constants = \
                            left_token.is_unit_propagation_based_on_constants or \
                            right_token.is_unit_propagation_based_on_constants
                    new_error.is_unit_propagation_based_on_unknown_variable = \
                            left_token.is_unit_propagation_based_on_unknown_variable or \
                            right_token.is_unit_propagation_based_on_unknown_variable
                    if new_error.is_unit_propagation_based_on_constants or \
                            new_error.is_unit_propagation_based_on_unknown_variable:
                        new_error.is_warning = True

                    if (not new_error.is_warning):
                        new_error.is_warning = self.check_if_error_with_low_confidence(token, left_token, right_token)

                    # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                    if self.source_file_exists:
                        pass
                    # COLLECT ERROR
                    self.all_errors.append(new_error)


    def error_check_logical_operators(self, sorted_analysis_unit_dict):
        ''' ERR CHECK UNITS DURING LOGICAL OPERATIONS
            input: sorted analysis unit dictionary of functions
            returns: none
            side_effects: might add UnitError objects to self.all_errors list
            '''
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(None)
            for root_token in function_dict['root_tokens']:
                tw.generic_recurse_and_apply_function(root_token, self.error_check_logical_recursive)

    
    def error_check_logical_recursive(self, token, left_token, right_token):
        if token.isOp and token.str in ['&&', '||', '!']:
            if (left_token and left_token.units) or (right_token and right_token.units):
                # UNIT ERROR ON LOGICAL OPERATION
                new_error = UnitError()
                new_error.ERROR_TYPE = UnitErrorTypes.LOGICAL_OPERATOR_USED_ON_UNITS
                # LINENR 
                new_error.linenr = token.linenr
                new_error.token_left = left_token
                new_error.token_right = right_token
                new_error.token = token
                new_error.var_name = self.get_var_name(left_token)
                new_error.is_unit_propagation_based_on_constants = token.is_unit_propagation_based_on_constants
                new_error.is_unit_propagation_based_on_unknown_variable = \
                        token.is_unit_propagation_based_on_unknown_variable
                if token.is_unit_propagation_based_on_constants or \
                        token.is_unit_propagation_based_on_unknown_variable:
                    new_error.is_warning = True
                # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                if self.source_file_exists:
                    pass
                # COLLECT ERROR
                self.all_errors.append(new_error)


    def check_if_error_with_low_confidence(self, token, left_token, right_token):
        #con.FOUND_DERIVED_CU_VARIABLE = False

        #units = self.get_left_right_units(token, left_token, right_token)

        #if con.FOUND_DERIVED_CU_VARIABLE:
        #    if len(units) > 2:
        #        return True
        #elif units:
        #    return True

        return False
        #return (True if units else False)


    def get_left_right_units(self, token, left_token, right_token):
        units = []
        left_units = []
        right_units = []

        if left_token:
            if left_token.str in ['*', '/'] and left_token.astOperand1 and left_token.astOperand2:
                if left_token.astOperand1.units == [{'nounit': 0.0}]:
                    left_token = left_token.astOperand2
                elif left_token.astOperand2.units == [{'nounit': 0.0}]:
                    left_token = left_token.astOperand1

            if left_token.str in ['+', '-', '*', '/']:
                left_units = self.get_left_right_units(left_token, left_token.astOperand1, left_token.astOperand2)
            else:
                left_units = left_token.units
                left_name = left_token.str
                if left_token.str == '.' or left_token.str == '[':
                    (left_token, left_name) = self.symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)

                if (left_token.variable, left_name) in con.variable2unitproba:
                    n = 3
                    if con.is_only_known_unit_variable(left_token.variable, left_name):
                        n = 1
                    left_units = con.variable2unitproba[(left_token.variable, left_name)][:n]
                    left_units = filter(lambda (u, p): p > con.unit_prob_threshold, left_units)
                    left_units = map(lambda (u, p): u, left_units)
                    if con.ENABLE_UNIT_LIST_FLATTENING:
                        left_units = con.flatten_unit_list(left_units)

        if right_token:
            if right_token.str in ['*', '/'] and right_token.astOperand1 and right_token.astOperand2:
                if right_token.astOperand1.units == [{'nounit': 0.0}]:
                    right_token = right_token.astOperand2
                elif right_token.astOperand2.units == [{'nounit': 0.0}]:
                    right_token = right_token.astOperand1

            if right_token.str in ['+', '-', '*', '/']:
                right_units = self.get_left_right_units(right_token, right_token.astOperand1, right_token.astOperand2)
            else:
                right_units = right_token.units
                right_name = right_token.str
                if right_token.str == '.' or right_token.str == '[':
                    (right_token, right_name) = self.symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                if (right_token.variable, right_name) in con.variable2unitproba:
                    n = 3
                    if con.is_only_known_unit_variable(right_token.variable, right_name):
                        n = 1
                    right_units = con.variable2unitproba[(right_token.variable, right_name)][:n]
                    right_units = filter(lambda (u, p): p > con.unit_prob_threshold, right_units)
                    right_units = map(lambda (u, p): u, right_units)
                    if con.ENABLE_UNIT_LIST_FLATTENING:
                        right_units = con.flatten_unit_list(right_units)
        
        if not left_units:
            return right_units
        elif not right_units:
            return left_units
        else:
            if token.str in ['*', '/']:
                tw = TreeWalker(None)
                all_unit_dicts_from_multiplication = []
                for unit_dict_left in left_units:
                    for unit_dict_right in right_units:
                        result_units = tw.apply_multiplication_to_unit_dicts(
                                unit_dict_left,
                                unit_dict_right,
                                token.str)
                        if result_units:
                            all_unit_dicts_from_multiplication.append(result_units)
                for u in all_unit_dicts_from_multiplication:
                    if u not in units:
                        units.append(u)
            else:
                for lu in left_units:
                    if lu in right_units:
                        units.append(lu)

        return units

    
    def check_errors_with_low_confidence_when_top3_units(self, cppcheck_configuration_unit, sorted_analysis_unit_dict):
        # need to work on another copy of cppcheckdata
        # check after all errors are collected

        con.print_known_unit_variables()
  
        data = cppcheckdata.parsedump(self.dump_file)
        for c in data.configurations[:1]:
            break

        # copy token and function data from original config
        tokenlist = {}

        for t in cppcheck_configuration_unit.tokenlist:
            tokenlist[t.Id] = (t.isRoot, t.isDimensionless)

        functionlist = {}
        returnexprtokenlist = {}

        for f in cppcheck_configuration_unit.functions:
            functionlist[f.Id] = (f.return_units, f.arg_units, f.return_arg_var_nr,
                                  f.return_expr_root_token,
                                  f.is_unit_propagation_based_on_constants,
                                  f.is_unit_propagation_based_on_unknown_variable,
                                  f.is_unit_propagation_based_on_weak_inference,
                                  f.maybe_generic_function)

            if f.return_expr_root_token:
                returnexprtokenlist[f.return_expr_root_token.Id] = None 


        returntokenlist = {}

        for t in c.tokenlist:
            (isRoot, isDimensionless) = tokenlist[t.Id]

            t.units = []
            t.isKnown = False
            t.is_unit_propagation_based_on_constants = False
            t.is_unit_propagation_based_on_unknown_variable = False
            t.is_unit_propagation_based_on_weak_inference = False
            t.isRoot = isRoot
            t.isDimensionless = isDimensionless

            if t.str == "return":
                returntokenlist[t.Id] = t

            if t.Id in returnexprtokenlist:
                returnexprtokenlist[t.Id] = t

        for f in c.functions:
            (return_units, arg_units, return_arg_var_nr, 
             return_expr_root_token, 
             is_unit_propagation_based_on_constants,
             is_unit_propagation_based_on_unknown_variable,
             is_unit_propagation_based_on_weak_inference,
             maybe_generic_function) = functionlist[f.Id]

            if return_expr_root_token:
                return_expr_root_token = returnexprtokenlist[return_expr_root_token.Id]

            f.return_units = [] #return_units
            f.arg_units = [] #arg_units
            f.return_arg_var_nr = return_arg_var_nr
            f.return_expr_root_token = return_expr_root_token
            f.is_unit_propagation_based_on_constants = is_unit_propagation_based_on_constants
            f.is_unit_propagation_based_on_unknown_variable = is_unit_propagation_based_on_unknown_variable
            f.is_unit_propagation_based_on_weak_inference = is_unit_propagation_based_on_weak_inference
            f.maybe_generic_function = maybe_generic_function
            for arg_number in f.argument.keys():
                f.arg_units.append([])


        # collect return units of all functions
        returnlist = {}

        for function_dict in sorted_analysis_unit_dict.values():
            if not function_dict['scopeObject'].function:
                continue
            if function_dict['scopeObject'].function.return_arg_var_nr:
                continue
            if function_dict['scopeObject'].function.maybe_generic_function:
                continue            

            returnlist[function_dict['scopeObject'].function.Id] = []

            for root_token in function_dict['root_tokens']:
                if root_token.str == 'return':
                    t = returntokenlist[root_token.Id]

                    self.check_error_when_top3_units(t)

                    #RETURN STATEMENT WITH UNITS - STORE UNITS
                    if t.units:
                        for u in t.units:
                            if u not in returnlist[function_dict['scopeObject'].function.Id]:
                                returnlist[function_dict['scopeObject'].function.Id].append(u)
                        
                    tw = TreeWalker(None)
                    tw.generic_recurse_and_apply_function(t, tw.reset_tokens)

        for f in c.functions:
            return_units = returnlist.get(f.Id)
            if return_units:
                f.return_units = return_units

        
        # check all errors
        for e in self.all_errors:
            con.FOUND_DERIVED_CU_VARIABLE = False

            if e.is_warning:
                continue

            if e.dont_check_for_warning:
                continue

            if e.ERROR_TYPE == UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS or \
                    e.ERROR_TYPE == UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS:

                root_token = None
                # find token in the copy
                for t in c.tokenlist:
                    if t.Id == e.token.Id:
                        root_token = t
                        break
                if not root_token:
                    continue

                self.check_error_when_top3_units(root_token)

                if e.ERROR_TYPE == UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS:
                    if con.FOUND_DERIVED_CU_VARIABLE:
                        if len(root_token.units) > 2:
                            e.is_warning = True
                    elif root_token.units:
                        e.is_warning = True

                else:
                    units = []
                    if root_token.astOperand1 and root_token.astOperand2:
                        left_units = root_token.astOperand1.units
                        right_units = root_token.astOperand2.units                  
                        if not left_units:
                            pass #units = right_units
                        elif not right_units:
                            pass #units = left_units
                        else:
                            for lu in left_units:
                                if lu in right_units:
                                    units.append(lu)
                    
                    if con.FOUND_DERIVED_CU_VARIABLE:
                        if len(units) > 2:
                            e.is_warning = True
                    elif units:
                        e.is_warning = True

                tw = TreeWalker(None)
                tw.generic_recurse_and_apply_function(root_token, tw.reset_tokens)

            elif e.ERROR_TYPE == UnitErrorTypes.VARIABLE_MULTIPLE_UNITS:
                
                i = 0
                root_token = None
                left_token = None
                # find token in the copy
                for t in c.tokenlist:
                    if t.Id == e.token.Id:
                        root_token = t
                        i = i + 1
                    elif t.Id == e.token_left.Id:
                        left_token = t
                        i = i + 1
                    if i == 2:
                        break
                if (not root_token) or (not left_token):
                    continue
                elif not root_token.astOperand2:
                    continue
                elif not root_token.astOperand1:
                    continue

                self.check_error_when_top3_units(root_token.astOperand1)
                self.check_error_when_top3_units(root_token.astOperand2)

                if (not left_token.isKnown): #and root_token.astOperand2.units:
                    if con.FOUND_DERIVED_CU_VARIABLE:
                        if len(root_token.astOperand2.units) > 2:
                            e.is_warning = True
                    elif root_token.astOperand2.units:
                        e.is_warning = True
                else:
                    if root_token.astOperand2.units and (root_token.astOperand1.units == root_token.astOperand2.units):
                        e.is_warning = True

                    if (not e.is_warning) and (root_token.astOperand1.is_unit_propagation_based_on_weak_inference):
                        units = []
                        for lu in root_token.astOperand1.units:
                            if lu in root_token.astOperand2.units:
                                units.append(lu)
                        
                        if con.FOUND_DERIVED_CU_VARIABLE:
                            if len(units) > 2:
                                e.is_warning = True
                        elif units:
                            e.is_warning = True
                        
 
                    #if not con.is_df_constraint_present(e.token_left, e.var_name):
                    #    if root_token.astOperand2.units: #and (root_token.astOperand1.units == root_token.astOperand2.units):
                    #        units = []
                    #        for lu in root_token.astOperand1.units:
                    #            if lu in root_token.astOperand2.units:
                    #                units.append(lu)
                        
                    #        if con.FOUND_DERIVED_CU_VARIABLE:
                    #            if len(units) > 2:
                    #                e.is_warning = True
                    #        elif units:
                    #            e.is_warning = True

                tw = TreeWalker(None)
                tw.generic_recurse_and_apply_function(root_token, tw.reset_tokens)


    def check_error_when_top3_units(self, root_token):
        tw = TreeWalker(None)  

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0

        tw.is_unit_propagation_based_on_constants = False
        tw.is_unit_propagation_based_on_unknown_variable = False

        # RESET THE TREE WALKER'S LINE NUMBERS
        tw.reset_min_max_line_numbers()
        # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
        tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)

        # APPLY ROS UNITS TO VARIABLES
        tw.generic_recurse_and_apply_function(root_token, tw.apply_ROS_units)
        # APPLY UNITS TO KNOWN SYMBOLS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_known_symbols_units)
        # APPLY UNITS FROM KNOWN FUNCTION - getX, getY, getZ  
        tw.generic_recurse_and_apply_function(root_token, tw.apply_units_getXYZ)
        # APPLY UNITS TO CONVERSION FACTORS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_conversion_factor_units)
        # APPLY FUNCTION RETURN UNITS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_function_return_units)
        # APPLY UNITS FROM PREVIOUS ROUND
        tw.generic_recurse_and_apply_function(root_token, tw.apply_previous_round_top3_units)
        # APPLY DIMENSIONLESS UNITS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_dimensionless_units)


        if (not tw.was_some_unit_changed):
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
            

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
            tw.perform_intersection = True
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
            tw.perform_intersection = False
            tw.perform_intersection = True
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_fmod_fmodf_fmodl)
            tw.perform_intersection = False
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
            # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
            tw.perform_intersection = True
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
            tw.perform_intersection = False
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
            tw.perform_intersection = True
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
            tw.perform_intersection = False
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
            tw.generic_recurse_and_apply_function(root_token, tw.collect_function_param_units_and_decorate_function)
            tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
        # END -- WHILE LOOP


    def get_var_name(self, token):
        name = ''
        if token.variable or token.str in ['.', '[']:
            name = self.symbol_helper.recursively_visit(token)
        if (not name) and token.astOperand1:
            name = self.get_var_name(token.astOperand1)   
        if (not name) and token.astOperand2:
            name = self.get_var_name(token.astOperand2)
        return name


    def collect_var_units_for_check(self, token, left_token, right_token):
        if token.variable and token.units:
            (token, var_name) = self.symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return

            if (token.variable, var_name) not in self.variable_units_to_check:
                self.variable_units_to_check[(token.variable, var_name)] = [token.isKnown, token.units, [token.linenr]]
            else:
                if token.linenr not in (self.variable_units_to_check[(token.variable, var_name)])[2]:
                    (self.variable_units_to_check[(token.variable, var_name)])[2].append(token.linenr)


    def print_unit_errors(self, errors_file, show_high_confidence=True, show_low_confidence=False):
        error_type_text = [
                           'VARIABLE_MULTIPLE_UNITS',
                           'COMPARISON_INCOMPATIBLE_UNITS',
                           'VARIABLE_BECAME_UNITLESS',
                           'FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS',
                           'VARIABLE_WITH_UNUSUAL_UNITS',
                           'ADDITION_OF_INCOMPATIBLE_UNITS',
                           'LOGICAL_OPERATOR_USED_ON_UNITS',
                           'UNIT_SMELL',
                          ]
        tw = TreeWalker(None)

        with open(errors_file, 'w') as f:
            for e in self.all_errors:
                is_high_confidence = not e.is_warning
                is_low_confidence = e.is_warning

                if is_high_confidence and not show_high_confidence:
                    continue
                if is_low_confidence and not show_low_confidence:
                    continue

                linenr = e.linenr
                etype = error_type_text[e.ERROR_TYPE]
                name = e.var_name
                f.write("%s, %s, %s\n" % (linenr, name, etype))

                if e.ERROR_TYPE == UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS:
                    tw.generic_recurse_and_apply_function(e.token_left, self.collect_var_units_for_check)
                    tw.generic_recurse_and_apply_function(e.token_right, self.collect_var_units_for_check)
                else:
                    tw.generic_recurse_and_apply_function(e.token, self.collect_var_units_for_check)


    def print_var_units_to_check(self, check_file):
        # convert var_units_for_check dictionary to a list ordered by ranking
        for (var, var_name) in self.variable_units_to_check:
            value = self.variable_units_to_check[(var, var_name)]
            isKnown = value[0]
            rank = 1.0
            if (var, var_name) in con.variable2unitproba:
                if len(con.variable2unitproba[(var, var_name)]) >= 2:
                    unit, proba = con.variable2unitproba[(var, var_name)][0]
                    unit2, proba2 = con.variable2unitproba[(var, var_name)][1]
                    rank = proba - proba2
            self.variable_units_to_check_as_list.append((isKnown, rank, var, var_name, value[1], value[2]))
        self.variable_units_to_check_as_list = sorted(self.variable_units_to_check_as_list, key=itemgetter(0, 1))
            
        with open(check_file, 'w') as f:
            for (isKnown, rank, var, var_name, units, linenrs) in self.variable_units_to_check_as_list:
                f.write("%s, %s, %s, %s, %s\n" % (linenrs[0], rank, var.Id, var_name, units))


    def pretty_print(self, show_high_confidence=True, show_low_confidence=False):
        ''' PRINTS ERRORS TO STD OUT, ATTEMPTS TO BE HUMAN READABLE
        '''
        for e in self.all_errors:
            is_high_confidence = not e.is_warning
            is_low_confidence = e.is_warning

            if is_high_confidence and not show_high_confidence:
                continue
            if is_low_confidence and not show_low_confidence:
                continue

            print '- '*42

            if e.is_warning:
                confidence = 'low'
            else:
                confidence = 'high'

            if e.ERROR_TYPE == UnitErrorTypes.VARIABLE_MULTIPLE_UNITS:
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                # MUTIPLE UNITS ON VARIABLE
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                print e.get_error_desc()
           
                units = []
                if e.units_when_multiple_happened:
                    units = e.units_when_multiple_happened

                print "Assignment of multiple units on line %s. Units: %s" % (e.linenr, units)
                #print e.all_units_assigned_to_var
                #print 'UNITS FIRST ASSIGNED LINE %s : %s' % (e.linenr_at_first_unit_assignment, e.units_at_first_assignment)
                #print 'SOURCE : %s' % e.source_code_at_first_assignment
                #print 'UNITS BECAME MULTIPLE AT LINE %s : %s' % \
                #        (e.linenr_of_multiple_unit_assignment, e.units_when_multiple_happened)
                #print 'SOURCE : %s' % e.source_code_when_multiple_units_happened
                #print
                #print 'Cause: A variable %s was assigned multiple units on line %s' % \
                #        (e.var_name, e.linenr_at_first_unit_assignment)
                # print '- '*42
            
            if e.ERROR_TYPE == UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS:
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                # FUNCTION CALLED WITH DIFFERENT UNIT ARGUMENTS
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                linenr_first = e.linenr_at_first_unit_assignment
                linenr_second = e.linenr_of_multiple_unit_assignment
                
                print
                print 'Cause: Function %s invoked with different units:' % e.var_name
                print 'LINE             UNITS'
                print '-----            -----------'
                for u in e.units_at_first_assignment:
                    print '{:5d}           '.format(linenr_first),
                    for k,v in u.iteritems():
                        print '{:10s} {:3f}  '.format(k, v), 
                    print
                for u in e.units_when_multiple_happened:
                    print '{:5d}           '.format(linenr_second),
                    for k,v in u.iteritems():
                        print '{:10s} {:3f}  '.format(k, v), 
                    print
                print
                print '{:5d} {:80s}'.format(linenr_first, e.source_code_at_first_assignment)
                print '{:5d} {:80s}'.format(linenr_second, e.source_code_when_multiple_units_happened)
                # print '- '*42

            if e.ERROR_TYPE == UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS:
                print e.get_error_desc()

                units_left = []
                if e.token_left and e.token_left.units:
                    units_left = e.token_left.units
                units_right = []
                if e.token_right and e.token_right.units:
                    units_right = e.token_right.units

                print "Addition of inconsistent units on line %s. Attempting to add %s to %s." % \
                        (e.linenr, units_left, units_right)

            if e.ERROR_TYPE == UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS:
                print e.get_error_desc()

                units_left = []
                if e.token_left and e.token_left.units:
                    units_left = e.token_left.units
                units_right = []
                if e.token_right and e.token_right.units:
                    units_right = e.token_right.units

                print "Comparison of inconsistent units on line %s. Attempting to compare %s to %s." % \
                        (e.linenr, units_left, units_right)


    def print_one_line_summary(self, show_high_confidence=True, show_low_confidence=True):
        """PRINTS A ONE LINE SUMMARY OF THE RESULTS FOR THIS ANALYSIS
        :show_high_confidence: boolean indicated whether to show high-confidence 
        :show_low_confidence: boolean indicated whether to show low-confidence 
        :returns: nothing.  Side-effect, prints summary to std outnothing.
        """
        sep = ', '
        s = ''
        # FILENAME
        s += 'file:' + self.current_file_under_analysis.replace('.dump','') + sep 
        # NUMBER OF STRONG INCONSISTENCIES
        count_strong = len([e for e in self.all_errors if not e.is_warning])
        # NUMBER OF WEAK INCONSISTENCIES
        count_weak = len([e for e in self.all_errors if e.is_warning])
        if count_strong > 0:
            s += "strong:%d," % count_strong
        if count_weak > 0:
            s += "weak:%d," % count_weak
        # TAKE THE WHOLE THING AND PRINT IT
        print (s)


