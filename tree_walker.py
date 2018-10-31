from symbol_helper import SymbolHelper
import cps_constraints as con
import copy
from operator import itemgetter


class TreeWalker:

    name = None

    def __init__(self, my_type_miner, my_vnh=None):
        self.type_miner = my_type_miner
        self.vnh = my_vnh
        self.my_symbol_helper = SymbolHelper()
        self.symbol_helper = self.my_symbol_helper
        self.source_file = ''
        self.source_file_lines = []
        self.current_file = ''
        self.current_function_dict = None
        self.return_value_list = []
        self.current_child_vars = []
        self.is_unit_propagation_based_on_weak_inference = False
        self.is_unit_propagation_based_on_unknown_variable = False
        self.is_unit_propagation_based_on_constants = False
        self.current_AST_start_line_number = None  # PROTECTS FROM MULTI-LINE STATEMENTS
        self.current_AST_end_line_number = None  # PROTECTS FROM MULTI-LINE STATEMENTS
        self.was_some_unit_changed = False
        self.found_units_in_this_tree = False
        self.MIN_VAR_NAME_LENGTH = 3
        self.perform_intersection = False
        self.perform_union_when_empty = False
        self.found_var = False
        self.found_arg_or_local_var = False
        self.found_arg_var = False
        self.found_non_arg_var = False
        self.found_non_known_unit_variable_in_rhs = False
        self.found_known_unit_variable_in_rhs = False
        self.found_non_ros_unit_variable_in_rhs = False


    def generic_recurse_and_apply_function(self, token, function_to_apply):
        ''' GENERIC RECURSION PATTERN - LEFT RIGHT TOKEN
            input:  token  CPPCHECK token to recurse upon
            function_to_apply:  the function to apply to the token 
                after recursing.
            returns: None   Side effect determined by function_to_apply
            '''
        if not token:
            return
        # INITIALIZE
        left_token = right_token = None

        # LEFT
        if token.astOperand1:
            left_token = token.astOperand1
            self.generic_recurse_and_apply_function(left_token,
                                                    function_to_apply)
        # RIGHT
        if token.astOperand2:
            right_token = token.astOperand2
            self.generic_recurse_and_apply_function(right_token,
                                                    function_to_apply)

        function_to_apply(token, left_token, right_token)


    def find_min_max_line_numbers(self, token, left_token, right_token):
        ''' FIND THE MIN AND MAX LINE NUMBERS FOR THIS AST,
                PROTECT FROM MULTI-LINE STATEMENTS
            input:  token AN AST TOKEN
                    left_token  (ignored)
                    right_token  (ignored)
            returns: None  (side effect: modifies class min and max
                line number range
            '''
        if not self.current_AST_start_line_number or \
                token.linenr < self.current_AST_start_line_number:
            self.current_AST_start_line_number = token.linenr
        if not self.current_AST_end_line_number or \
                token.linenr > self.current_AST_end_line_number:
            self.current_AST_end_line_number = token.linenr


    def reset_min_max_line_numbers(self):
        ''' INITIALIZES THE AST LINE NUMBERS BACK TO NONE.
                SHOULD BE CALLED BEFORE EVERY AST IS EVALUATED
            input: None
            output: None.  side effect is setting class variables
                for min max to None
            '''
        self.current_AST_start_line_number = None
        self.current_AST_end_line_number = None


    def reset_tokens(self, token, left_token, right_token):
        if token:
            token.units = []
            token.isKnown = False
            token.is_unit_propagation_based_on_constants = False
            token.is_unit_propagation_based_on_unknown_variable = False
            token.is_unit_propagation_based_on_weak_inference = False


    def scan_variables(self, token, left_token, right_token):
        if token.variable:
            self.found_var = True
            if token.variable.isArgument or token.variable.isLocal:
                self.found_arg_or_local_var = True
            if token.variable.isArgument:
                self.found_arg_var = True
            if not token.variable.isArgument:
                self.found_non_arg_var = True


    def apply_previous_round_top3_units(self, token, left_token, right_token):
        if token.variable and (not token.units):
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return
            #TODO improve by storing variable id in the datastructures instead of variable object
            for (token_variable, name) in con.variable2unitproba:
                if (token_variable.Id == token.variable.Id) and (name == var_name):
                    n = 3
                    if con.is_only_known_unit_variable(token_variable, name):
                        n = 1

                    #units = con.variable2unitproba[(token_variable, var_name)][:n]
                    #units = filter(lambda (u, p): p > con.unit_prob_threshold, units)
                    #units = map(lambda (u, p): u, units)
                    #if con.ENABLE_UNIT_LIST_FLATTENING:
                    #    units = con.flatten_unit_list(units)

                    units = con.variable2unitproba[(token_variable, var_name)]
                    units = filter(lambda (u, p): p > con.unit_prob_threshold, units)
                    probas = map(lambda (u, p): p, units)
                    probas = list(set(probas))
                    probas = sorted(probas, reverse=True)
                    probas = probas[:n]
                    units = filter(lambda (u, p): p in probas, units)
                    units = map(lambda (u, p): u, units)
                    if con.ENABLE_UNIT_LIST_FLATTENING:
                        units = con.flatten_unit_list(units)

                    token.units = units
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    break


    def apply_correction_units(self, token, left_token, right_token):
        if token.variable:
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return
            if var_name in con.phys_corrections:
                token.units = (con.phys_corrections)[var_name]
                if token.units == [{'dimensionless': 1.0}]:
                    token.units = []
                self.was_some_unit_changed = True
                self.found_units_in_this_tree = True


    def apply_ROS_units(self, token, left_token, right_token):
        ''' DECORATE LEAF WITH ROS MESSAGE TYPE UNITS
            input:  token AN AST TOKEN
                    left_token  (ignored)
                    right_token  (ignored)
            returns: None  (side effect: adds units to ROS variable leafs)
        '''
        if token.variable:
            # LOOKUP VARIABLE TYPE IN SYMBOL DICTIONARY FOR ROS MESSAGES
            units_as_dict = self.my_symbol_helper.find_units_for_variable(token)
            # ADD THIS DICTIONARY
            if units_as_dict and (units_as_dict not in token.units):
                token.units.append(units_as_dict)
                token.isKnown = True
                self.was_some_unit_changed = True
                self.found_units_in_this_tree = True
                
                if (token.str != 'dt'):
                    con.found_ros_units = True
                else:
                    token.isKnown = False
                
                # CHECK SYMBOL HELPER FOR WEAK INFERENCE IN THE CASE OF
                # JOINT_STATES AND getXYZ()
                if self.my_symbol_helper.is_weak_inference:
                    self.is_unit_propagation_based_on_weak_inference = True
                    token.is_unit_propagation_based_on_weak_inference = True


    def apply_known_symbols_units(self, token, left_token, right_token):
        a_dict = None

        if token.str in ['atan2', 'acos', 'asin', 'atan',  
                         'M_PI', 'toSec', 'toNSec', 'getYaw', 'getRoll', 'getPitch']:
            if token.str in self.my_symbol_helper.ros_unit_dictionary:
                a_dict = self.my_symbol_helper.ros_unit_dictionary[token.str][token.str]

        if token.str in ['cos', 'sin', 'tan']:
            a_dict = {'nounit': 0.0}

        if a_dict and (a_dict not in token.units):
            token.units.append(a_dict)
            token.isKnown = True
            self.was_some_unit_changed = True
            self.found_units_in_this_tree = True


    def apply_units_getXYZ(self, token, left_token, right_token):
        if token.str not in ['getX', 'getY', 'getZ']:
            return

        units_for_getXYZ = []

        # TRY TO FIGURE OUT TYPE THIS WAS CALLED ON
        # ASSUME PARENT TOKEN IS '.'
        # OPTION 1 - LOOK FOR getOrigin and getRotation
        if token.astParent.astOperand1:
            if token.astParent.astOperand1.str == '(':
                paren_token = token.astParent.astOperand1
                if paren_token.astOperand1 and paren_token.astOperand1.str == '.':
                    dot_token = paren_token.astOperand1
                    if dot_token.astOperand1 and dot_token.astOperand1.variable and dot_token.astOperand2:
                        class_name = self.my_symbol_helper.find_variable_type(dot_token.astOperand1.variable)
                        class_name_sanitized = self.my_symbol_helper.sanitize_class_name(class_name)
                        if class_name_sanitized in self.my_symbol_helper.ros_unit_dictionary:
                            if dot_token.astOperand2.str == 'getOrigin':
                                units_for_getXYZ = self.my_symbol_helper.ros_unit_dictionary[class_name_sanitized]['getOrigin']
                            elif dot_token.astOperand2.str == 'getRotation':
                                units_for_getXYZ = self.my_symbol_helper.ros_unit_dictionary[class_name_sanitized]['getRotation']
        
            # GET TYPE SO WE CAN INFER CORRECT UNITS BASED ON KNOWLEDGE OF ROS UNIT ASSUMPTIONS
            elif token.astParent.astOperand1.variable:
                class_name = self.my_symbol_helper.find_variable_type(token.astParent.astOperand1.variable)
                class_name_sanitized = self.my_symbol_helper.sanitize_class_name(class_name)
                if class_name_sanitized in self.my_symbol_helper.ros_unit_dictionary:
                    units_for_getXYZ = self.my_symbol_helper.ros_unit_dictionary[class_name_sanitized][token.str]
        else:
            # NO AST PARENT OPERAND1 - SHOULD BE IMPOSSIBLE
            # assert False
            pass

        if units_for_getXYZ and (units_for_getXYZ not in token.units):
            token.units.append(units_for_getXYZ)
            token.isKnown = True
            self.was_some_unit_changed = True
            self.found_units_in_this_tree = True
            

    def apply_conversion_factor_units(self, token, left_token, right_token):
        # HANDLE DEGREE FROM/TO RADIAN CONVERSION
        a_dict = None

        if token.isArithmeticalOp and (token.str == '/'):
            if right_token.str in ['180', '180.0', '180.0f']:
                if left_token.str[:4] in ['M_PI', '3.14']:
                    a_dict = {'radian_unit': 1.0}
                    token = left_token
                elif left_token.str == '*' and (left_token.astOperand2.str[:4] in ['M_PI', '3.14'] or \
                        left_token.astOperand1.str[:4] in ['M_PI', '3.14']):
                    token = right_token
                    a_dict = {'radian_unit': 1.0} 
            elif right_token.str[:4] in ['M_PI', '3.14']:
                if left_token.str in ['180', '180.0', '180.0f']:
                    a_dict = {'degree_360_unit': 1.0}
                elif left_token.str == '*' and (left_token.astOperand2.str in ['180', '180.0', '180.0f'] or \
                        left_token.astOperand1.str in ['180', '180.0', '180.0f']):
                    token = right_token
                    a_dict = {'degree_360_unit': 1.0}
        
        elif token.isArithmeticalOp and (token.str == '*'):
            if right_token and right_token.str[:4] in ['M_PI', '3.14']:
                if left_token.str == '/' and left_token.astOperand2.str in ['180', '180.0', '180.0f']:
                    token = left_token.astOperand2
                    a_dict = {'radian_unit': 1.0}
            elif right_token and right_token.str in ['180', '180.0', '180.0f']:
                if left_token.str == '/' and left_token.astOperand2.str[:4] in ['M_PI', '3.14']:
                    token = left_token.astOperand2
                    a_dict = {'degree_360_unit': 1.0}
            
        if a_dict and (a_dict not in token.units):
            token.units.append(a_dict)
            self.was_some_unit_changed = True
            self.found_units_in_this_tree = True   


    def apply_function_return_units(self, token, left_token, right_token):
        if token.function:
            function_return_units = token.function.return_units
            for u in function_return_units:
                if u not in token.units:
                    token.units.append(u)
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    self.is_unit_propagation_based_on_constants = \
                            token.function.is_unit_propagation_based_on_constants
                    token.is_unit_propagation_based_on_constants = \
                            token.function.is_unit_propagation_based_on_constants
                    self.is_unit_propagation_based_on_unknown_variable = \
                            token.function.is_unit_propagation_based_on_unknown_variable
                    token.is_unit_propagation_based_on_unknown_variable = \
                            token.function.is_unit_propagation_based_on_unknown_variable
                    self.is_unit_propagation_based_on_weak_inference = \
                            token.function.is_unit_propagation_based_on_weak_inference
                    token.is_unit_propagation_based_on_weak_inference = \
                            token.function.is_unit_propagation_based_on_weak_inference


    def apply_previous_round_units(self, token, left_token, right_token):
        if token.variable and (not token.units):
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return
            if (token.variable, var_name) in con.variable2unitproba:
                if len(con.variable2unitproba[(token.variable, var_name)]) >= 2:
                    unit, proba = con.variable2unitproba[(token.variable, var_name)][0]
                    unit2, proba2 = con.variable2unitproba[(token.variable, var_name)][1]
                else:
                    unit, proba = con.variable2unitproba[(token.variable, var_name)][0]
                    proba2 = 0.0
                proba = round(proba, 7)
                proba2 = round(proba2, 7)
                if (proba > con.unit_prob_threshold) and (unit not in token.units) and (proba != proba2):
                    #print var_name, unit, proba
                    if isinstance(unit, list):
                        token.units = unit
                    else:
                        token.units.append(unit)
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True            


    def apply_dimensionless_units(self, token, left_token, right_token):
        if token.variable:
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return
            if (token.variable, var_name) in con.dimensionless_variables:
                token.units = []
                token.isDimensionless = True
                self.was_some_unit_changed = True
                self.found_units_in_this_tree = True
    
  
    def collect_implied_non_unit_variables(self, token, left_token, right_token):
        if token.isOp and token.str in ['=', '<', '<=', '==', '!=', '>', '>=']:
            if left_token and right_token:
                
                left_name = left_token.str
                right_name = right_token.str
                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(' :
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                lunit = self.my_symbol_helper.should_have_unit(left_token, left_name)
                runit = self.my_symbol_helper.should_have_unit(right_token, right_name)

                if left_token.variable and right_token.variable:
                    ltype = self.my_symbol_helper.find_variable_type(left_token.variable)
                    ltype = ltype.lower()
                    rtype = self.my_symbol_helper.find_variable_type(right_token.variable)
                    rtype = rtype.lower()
                    if (lunit and ('int' in rtype)):
                        con.add_int_unit_variable(right_token, right_name)
                        return
                    if (runit and ('int' in ltype)):
                        con.add_int_unit_variable(left_token, left_name)
                        return
                        
                if (not lunit) and (runit):
                    if right_token.variable:
                        con.add_non_unit_variable(right_token, right_name)
                elif (lunit) and (not runit):
                    if left_token.variable:
                        con.add_non_unit_variable(left_token, left_name)      


    def add_df_constraint(self, left_token, left_name, right_token, right_name, df_type=con.DF_1):
        if (left_token.variable == right_token.variable) and (left_name == right_name):
            return

        if left_token.is_unit_propagation_based_on_weak_inference or right_token.is_unit_propagation_based_on_weak_inference:
            return

        b1 = self.my_symbol_helper.should_have_unit(left_token, left_name)
        b2 = self.my_symbol_helper.should_have_unit(right_token, right_name)
        if (b1 and b2): 
            con.add_df_constraint(left_token, left_name, right_token, right_name, df_type)


    def collect_same_unit_constraints(self, token, left_token, right_token):
        if token.isOp and token.str in ['+', '-', '+=', '-=', '=', '<', '<=', '==', '!=', '>', '>=']:
            #print token.str, token.linenr
            if left_token and right_token:
                #print token.str, token.linenr, left_token.str, right_token.str
                left_name = left_token.str
                right_name = right_token.str

                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)                

                # COLLECT CONSTRAINT
                if left_token.variable or right_token.variable:
                    token.hasVarOperand = True
                elif left_token.hasVarOperand or right_token.hasVarOperand:
                    token.hasVarOperand = True

                if left_token.variable and right_token.variable:
                    self.add_df_constraint(left_token, left_name, right_token, right_name)

                    if (not token.astParent) or (token.astParent and (token.astParent.str not in ['(', '['])):
                        self.current_child_vars.append((left_token, left_name))
                        self.current_child_vars.append((right_token, right_name))

                elif left_token.variable:
                     if token.str in ['+=', '-=', '=', '+', '-', '<', '<=', '==', '!=', '>', '>=']:
                         if (right_name in ['+', '-', '?', '(', '*', '/']) and right_token.hasVarOperand:
                             for var in self.current_child_vars:
                                 right_token, right_name = var
                                 self.add_df_constraint(left_token, left_name, right_token, right_name)
                     
                         elif right_token.isKnown and right_token.units:
                             if token.str in ['+', '-', '<', '<=', '==', '!=', '>', '>=']:
                                 self.add_ks_constraint(left_token, left_name, right_token.units)

                     if (not token.astParent) or (token.astParent and (token.astParent.str not in ['(', '['])):
                         self.current_child_vars.append((left_token, left_name))

                elif right_token.variable:
                     if token.str in ['+', '-', '<', '<=', '==', '!=', '>', '>=']:
                         if (left_name in ['+', '-', '(', '*', '/']) and left_token.hasVarOperand:
                             for var in self.current_child_vars:
                                 left_token, left_name = var
                                 self.add_df_constraint(left_token, left_name, right_token, right_name)
                         
                         elif left_token.isKnown and left_token.units:
                             self.add_ks_constraint(right_token, right_name, left_token.units)

                     if (not token.astParent) or (token.astParent and (token.astParent.str not in ['(', '['])):
                         self.current_child_vars.append((right_token, right_name))
        
            elif left_token and token.str == '-':
                # UNARY MINUS
                left_name = left_token.str
                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                #print token.str, token.linenr, left_token.str, left_name

                if left_token.variable:
                    token.hasVarOperand = True
                    self.current_child_vars.append((left_token, left_name))
                elif left_token.hasVarOperand:
                    token.hasVarOperand = True

        elif token.str == '?':
            if right_token.str == ':':
                # TERNARY OPERATOR ?:
                if left_token.hasVarOperand:
                    self.current_child_vars = []

                left_token = right_token.astOperand1
                left_name = left_token.str
                right_token = right_token.astOperand2
                right_name = right_token.str

                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                if left_token.variable or right_token.variable:
                    token.hasVarOperand = True
                    if left_token.variable:
                        self.current_child_vars.append((left_token, left_name))
                    if right_token.variable:
                        self.current_child_vars.append((right_token, right_name))
                    if left_token.variable and right_token.variable:
                        self.add_df_constraint(left_token, left_name, right_token, right_name);

        elif token.str in ['min', 'max']:
            unit_receiver = None
            comma_token = None

            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    if unit_receiver.astOperand2 and unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2 and token.astParent.astOperand2.str == ',':
                    comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not unit_receiver or not comma_token:
                return

            left_token = comma_token.astOperand1
            right_token = comma_token.astOperand2
            if not (left_token and right_token):
                return

            left_name = left_token.str
            right_name = right_token.str

            if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
            if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                (right_token, right_name) = \
                        self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

            # COLLECT CONSTRAINT
            if left_token.variable or right_token.variable:
                unit_receiver.hasVarOperand = True

            if left_token.variable and right_token.variable:
                self.add_df_constraint(left_token, left_name, right_token, right_name)
                self.current_child_vars.append((left_token, left_name))
                self.current_child_vars.append((right_token, right_name))
            elif left_token.variable:
                if right_token.hasVarOperand:
                    for var in self.current_child_vars:
                        right_token, right_name = var
                        self.add_df_constraint(left_token, left_name, right_token, right_name)
                self.current_child_vars.append((left_token, left_name))
            elif right_token.variable:
                if left_token.hasVarOperand:
                    for var in self.current_child_vars:
                        left_token, left_name = var
                        self.add_df_constraint(left_token, left_name, right_token, right_name)
                self.current_child_vars.append((right_token, right_name))

        elif token.str in ['abs', 'fabs', 'floor', 'ceil']:
            unit_receiver = None
            right_token = None

            # CASE: NAMESPACE
            if token.astParent.str == '::' and token.astParent.astParent:
                unit_receiver = token.astParent.astParent
                if unit_receiver.astOperand2:
                    right_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    right_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not right_token or not unit_receiver:
                return

            right_name = right_token.str
            if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                (right_token, right_name) = \
                        self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

            if right_token.variable:
                unit_receiver.hasVarOperand = True
                self.current_child_vars.append((right_token, right_name))
            elif right_token.hasVarOperand:
                unit_receiver.hasVarOperand = True

        elif token.isOp and token.str in ['*', '/'] and (not token.is_unit_propagation_based_on_constants):
            nounit = True
            if [{'nounit': 0.0}] == left_token.units:
                left_token = right_token
            elif right_token and ([{'nounit': 0.0}] == right_token.units):
                left_token = left_token
            else:
                nounit = False

            if nounit:
                left_name = left_token.str
                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)

                if left_token.variable:
                    token.hasVarOperand = True
                    self.current_child_vars.append((left_token, left_name))
                elif left_token.hasVarOperand:
                    token.hasVarOperand = True
            else:
                self.current_child_vars = []

    
    def collect_same_unit_constraints_II(self, token, left_token, right_token):
        if token.isOp and token.str in ['+', '-', '<', '<=', '==', '!=', '>', '>=']:
            left_nounit = False
            right_nounit = False

            if left_token.str in ['+', '-']:
                if left_token.astOperand1 and (left_token.astOperand1.str in ['*', '/']):
                    left_token = left_token.astOperand1
                if left_token.astOperand2 and (left_token.astOperand2.str in ['*', '/']):
                    left_token = left_token.astOperand2

            #IF EXPR LIKE A*COS(P)+B*SIN(Q)
            if left_token.str in ['*', '/']:
                if [{'nounit': 0.0}] == left_token.astOperand1.units:
                    left_token = left_token.astOperand2
                    left_nounit = True
                elif left_token.astOperand2 and ([{'nounit': 0.0}] == left_token.astOperand2.units):
                    left_token = left_token.astOperand1
                    left_nounit = True
            
            
            if right_token and right_token.str in ['*', '/']:
                if [{'nounit': 0.0}] == right_token.astOperand1.units:
                    right_token = right_token.astOperand2
                    right_nounit = True
                elif right_token.astOperand2 and ([{'nounit': 0.0}] == right_token.astOperand2.units):
                    right_token = right_token.astOperand1
                    right_nounit = True

            if left_nounit and right_nounit:
                #HANDLE UNARY MINUS
                if left_token.isOp and left_token.str == '-' and (not left_token.astOperand2):
                    left_token = left_token.astOperand1
                if right_token.isOp and right_token.str == '-' and (not right_token.astOperand2):
                    right_token = right_token.astOperand1

                #COLLECT CONSTRAINT
                left_name = left_token.str
                right_name = right_token.str

                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = \
                    self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                    self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token) 
                    
                if left_token.variable and right_token.variable:
                    self.add_df_constraint(left_token, left_name, right_token, right_name)

        if token.isOp and token.str == '=':
            if right_token and right_token.isOp:
                op1 = None
                op2 = None 
                #AVERAGE EXPR
                if right_token.str == '/' and right_token.astOperand1.str == '+' and right_token.astOperand2.str in ['2', '2.0']:
                    op1 = right_token.astOperand1.astOperand1
                    op2 = right_token.astOperand1.astOperand2
                elif right_token.str == '*' and right_token.astOperand1 and right_token.astOperand2:
                    if right_token.astOperand1.str == '+' and right_token.astOperand2.str == '0.5':
                        op1 = right_token.astOperand1.astOperand1
                        op2 = right_token.astOperand1.astOperand2
                    elif right_token.astOperand1.str == '0.5' and right_token.astOperand2.str == '+':
                        op1 = right_token.astOperand2.astOperand1
                        op2 = right_token.astOperand2.astOperand2

                if op1 and op2:
                    #COLLECT CONSTRAINT
                    op1_name = op1.str
                    op2_name = op2.str

                    if op1.str == '.' or op1.str == '[' or op1.str == '(':
                        (op1, op1_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(op1)
                    if op2.str == '.' or op2.str == '[' or op2.str == '(':
                        (op2, op2_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(op2)

                    if op1.variable and op2.variable:
                        left_name = left_token.str
                        if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                            (left_token, left_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)

                        if left_token.variable:
                            self.add_df_constraint(left_token, left_name, op1, op1_name)
                            self.add_df_constraint(left_token, left_name, op2, op2_name)


    def collect_same_unit_constraints_III(self, token, left_token, right_token):
        if token.function: #and (not token.function.maybe_generic_function):
            paren_token = token.astParent
            while paren_token.str != '(':
                paren_token = paren_token.astParent

            function_args = []
            if paren_token.astOperand2:
                if paren_token.astOperand2.str == ',':
                    function_args = self.get_function_args(paren_token.astOperand2)
                else:
                    function_args = [paren_token.astOperand2]
            if function_args:
                if len(function_args) != len(token.function.argument):
                    return
                #print token.function.name, function_args
                for i, t in enumerate(function_args):
                    if t.str in ['&']:
                        t = t.astOperand1

                    t_name = t.str
                    if t.str == '.' or t.str == '[' or t.str == '(':
                        (t, t_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(t)

                    if t.variable:
                        var = token.function.argument[str(i+1)]
                        if var and var != t.variable:
                            self.add_df_constraint(t, t_name, var.nameToken, var.nameToken.str, con.DF_2)
                            

    #TODO similar to the function 'self.recurse_on_function_args'
    def get_function_args(self, comma_token):
        my_return_list = []
        if comma_token.astOperand1:
            if comma_token.astOperand1.str == ',':
                left_branch_list = self.get_function_args(comma_token.astOperand1)
                my_return_list.extend(left_branch_list)
            else:
                my_return_list.append(comma_token.astOperand1)
        if comma_token.astOperand2:
            my_return_list.append(comma_token.astOperand2)
        return my_return_list

                     
    def collect_lhs_unit_constraint(self, root_token, check_known_unit_variable=True):
        if root_token.astOperand1:
            #if (not root_token.astOperand1.units):

                if root_token.astOperand2 and root_token.astOperand2.str == '=':
                    #AVOID INITIALIZATION STATEMENTS e.g. x = y = 0
                    return

                if root_token.astOperand2 and root_token.astOperand2.units:
                    # AVOID CONSTRAINTS FOR WEAK UNITS
                    if root_token.astOperand2.is_unit_propagation_based_on_unknown_variable:
                        return
                    #if root_token.astOperand2.is_unit_propagation_based_on_constants:
                    #    return

                    if ({'degree_360_unit': -1.0} in root_token.astOperand2.units) or \
                           ({'radian_unit': -1.0} in root_token.astOperand2.units): #or \
                           #({'nounit': 0.0} in root_token.astOperand2.units):
                        return

                    # COLLECT LEFT SIDE OF ASSIGNMENT
                    lhs_var_token = root_token.astOperand1
                    lhs_name = root_token.astOperand1.str

                    if lhs_name == '.' or lhs_name == '[' or lhs_name == '(':
                        (lhs_var_token, lhs_name) = \
                        self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(lhs_var_token)
                    if (not lhs_var_token) or (not lhs_var_token.variable):
                        return

                    if not (self.my_symbol_helper.should_have_unit(lhs_var_token, lhs_name)):
                        return

                    if ({'nounit': 0.0} in root_token.astOperand2.units):
                        if root_token.astOperand2.units == [{'nounit': 0.0}]:
                            con.add_dimensionless_variable(lhs_var_token, lhs_name)
                        return

                    # SCAN RHS VARIABLES
                    self.found_non_known_unit_variable_in_rhs = False
                    self.found_known_unit_variable_in_rhs = False
                    self.found_non_ros_unit_variable_in_rhs = False
                    self.generic_recurse_and_apply_function(root_token.astOperand2, self.scan_rhs_variables)

                    k = (not self.found_non_known_unit_variable_in_rhs) and (self.found_known_unit_variable_in_rhs)
                    uk = (not self.found_known_unit_variable_in_rhs)

                    if (not lhs_var_token.isKnown):
                        con.add_known_unit_variable(lhs_var_token, lhs_name, k, uk)

                    # TRACK VARIABLE WITH MULTIPLE UNITS
                    if len(root_token.astOperand2.units) > 1:              
                        con.add_multi_unit_variable(root_token, lhs_var_token, lhs_name, root_token.astOperand2.units, k)

                    if lhs_var_token.isKnown:
                        if (len(root_token.astOperand2.units) == 1) and (lhs_var_token.units != root_token.astOperand2.units):
                            units = []
                            units.extend(lhs_var_token.units)
                            units.extend(root_token.astOperand2.units)
                            con.add_multi_unit_variable(root_token, lhs_var_token, lhs_name, units, k)
                        return
                
                    # COLLECT CONSTRAINT
                    isKnown = root_token.astOperand2.isKnown and (not self.found_non_ros_unit_variable_in_rhs) 
                    con.add_cu_constraint(lhs_var_token, lhs_name, root_token.astOperand2.units, isKnown)

                    # ChECK WHEN LHS IS KNOWN UNIT VARIABLE
                    if (check_known_unit_variable) and (con.is_only_known_unit_variable(lhs_var_token.variable, lhs_name)):
                        self.process_lhs_known_unit_variable(root_token, 
                                                             lhs_var_token, lhs_name, k,                 
                                                             root_token.astOperand2.units)

                elif root_token.astOperand2: #and root_token.astOperand2.isDimensionless:
                    # COLLECT LEFT SIDE OF ASSIGNMENT
                    lhs_var_token = root_token.astOperand1
                    lhs_name = root_token.astOperand1.str

                    if lhs_name == '.' or lhs_name == '[' or lhs_name == '(':
                        (lhs_var_token, lhs_name) = \
                        self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(lhs_var_token)
                    if (not lhs_var_token) or (not lhs_var_token.variable):
                        return

                    if not (self.my_symbol_helper.should_have_unit(lhs_var_token, lhs_name)):
                        return

                    if lhs_var_token.isKnown:
                        return

                    if root_token.astOperand2.isDimensionless:
                        con.add_dimensionless_variable(lhs_var_token, lhs_name)
                    
                    # SCAN RHS VARIABLES
                    self.found_non_known_unit_variable_in_rhs = False
                    self.found_known_unit_variable_in_rhs = False
                    self.found_non_ros_unit_variable_in_rhs = False
                    self.generic_recurse_and_apply_function(root_token.astOperand2, self.scan_rhs_variables)

                    k = (not self.found_non_known_unit_variable_in_rhs) and (self.found_known_unit_variable_in_rhs)
                    uk = (not self.found_known_unit_variable_in_rhs)

                    con.add_known_unit_variable(lhs_var_token, lhs_name, k, uk)
            #else:
            #    pass


    def scan_rhs_variables(self, token, left_token, right_token):
        if token.variable:
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            if not token:
                return

            if (not token.isKnown) and (not con.is_only_known_unit_variable(token.variable, var_name)):
                self.found_non_known_unit_variable_in_rhs = True

            if (token.isKnown) or (con.is_only_known_unit_variable(token.variable, var_name)):
                self.found_known_unit_variable_in_rhs = True

            if (not token.isKnown) and (self.my_symbol_helper.should_have_unit(token, var_name)):
                self.found_non_ros_unit_variable_in_rhs = True 


    def process_lhs_known_unit_variable(self, root_token, lhs_var_token, lhs_name, k, rhs_units):
        # TRACK VARIABLE WITH MULTIPLE UNITS
        if (len(rhs_units) == 1) and (lhs_var_token.units) and (lhs_var_token.units != rhs_units):
            units = []
            units.extend(lhs_var_token.units)
            units.extend(rhs_units)
            #con.add_multi_unit_variable(root_token, lhs_var_token, lhs_name, units, k)

        # PROCESS CU CONSTRAINTS
        con.scan_and_create_cu_constraints(lhs_var_token, lhs_name)


    def add_ks_constraint(self, token, name, units):
        if not (self.my_symbol_helper.should_have_unit(token, name)):
            return
        con.add_ks_constraint(token, name, units)


    def collect_known_symbol_constraints(self, token, left_token, right_token):
        if token.str in ['cos', 'sin']:
            if token.astParent and token.astParent.str == '(' and token.astParent.astOperand2:
                token = token.astParent.astOperand2
                name = token.str

                if token.str == '.' or token.str == '[' or token.str == '(':
                    (token, name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(token)

                # COLLECT CONSTRAINT
                if token.variable:
                    self.add_ks_constraint(token, name, [{'radian': 1.0}])


    def add_cf_constraint(self, token, name, units, cf_type=con.CF_1):
        if not (self.my_symbol_helper.should_have_unit(token, name)):
            return

        if (cf_type == con.CF_1) and ({'second': -1.0} in token.units):
            if units == [{'degree_360': 1.0}]:
                units = [{'degree_360': 1.0, 'second': -1.0}]
            elif units == [{'radian': 1.0}]:
                units = [{'second': -1.0}]
        
        con.add_cf_constraint(token, name, units, cf_type)


    #TODO handle all cases
    def collect_conversion_factor_constraints(self, token, left_token, right_token):
        #if token.str in ['=', '+=', '-=']:
            if (right_token and right_token.str == '/'and right_token.astOperand2) and \
                    (({'degree_360_unit': 1.0} in right_token.astOperand2.units) or \
                     ({'radian_unit': 1.0} in right_token.astOperand2.units)):

                top_right_token_unit = right_token.astOperand2.units
                right_token = right_token.astOperand1
                if right_token.str == '*':
                    if (right_token.astOperand2.str[:4] in ['M_PI', '3.14']) or \
                            (right_token.astOperand2.str in ['180', '180.0', '180.0f']):
                        right_token = right_token.astOperand1
                    else:
                        right_token = right_token.astOperand2
                
                right_name = right_token.str
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                    self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)
                 
                if (right_token.variable) and (not right_token.isKnown): #(not right_token.units):
                    unit = [{'radian': 1.0}] if (top_right_token_unit == [{'degree_360_unit': 1.0}]) else [{'degree_360': 1.0}]
                    self.add_cf_constraint(right_token, right_name, unit)

                if token.str not in ['=', '+=', '-=']:
                    return

                left_name = left_token.str
                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)

                #if (not left_token.units):
                if (not left_token.isKnown):
                    unit = [{'degree_360': 1.0}] if (top_right_token_unit == [{'degree_360_unit': 1.0}]) else [{'radian': 1.0}]
                    self.add_cf_constraint(left_token, left_name, unit)

            elif (right_token and right_token.str == '*' and right_token.astOperand1 and right_token.astOperand2): 

                if (({'radian_unit': 1.0} in right_token.astOperand1.units) or \
                    ({'degree_360_unit': 1.0} in right_token.astOperand1.units)):                    
                    top_right_token_unit = right_token.astOperand1.units
                    right_token = right_token.astOperand2
                
                elif (right_token.astOperand1.str == '/'and right_token.astOperand1.astOperand2) and \
                        (({'degree_360_unit': 1.0} in right_token.astOperand1.astOperand2.units) or \
                         ({'radian_unit': 1.0} in right_token.astOperand1.astOperand2.units)):
                    top_right_token_unit = right_token.astOperand1.astOperand2.units
                    right_token = right_token.astOperand1.astOperand1

                else:
                    return

                right_name = right_token.str
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                    self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                #if (not right_token.units):
                if (right_token.variable) and (not right_token.isKnown):
                    unit = [{'radian': 1.0}] if (top_right_token_unit == [{'degree_360_unit': 1.0}]) else [{'degree_360': 1.0}]
                    self.add_cf_constraint(right_token, right_name, unit)

                if token.str not in ['=', '+=', '-=']:
                    return

                left_name = left_token.str
                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)

                #if (not left_token.units):
                if (not left_token.isKnown):
                    unit = [{'degree_360': 1.0}] if (top_right_token_unit == [{'degree_360_unit': 1.0}]) else [{'radian': 1.0}] 
                    self.add_cf_constraint(left_token, left_name, unit)


    def collect_angle_unit_constraints(self, token, left_token, right_token):
        if token.isOp and token.str in ['+', '-', '+=', '-=', '=', '<', '<=', '==', '!=', '>', '>=']:
            if left_token and right_token:
                #HANDLE UNARY MINUS
                if left_token.isOp and left_token.str == '-' and (not left_token.astOperand2):
                    left_token = left_token.astOperand1
                if right_token.isOp and right_token.str == '-' and (not right_token.astOperand2):
                    right_token = right_token.astOperand1

                left_name = left_token.str
                right_name = right_token.str

                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                if (left_token.variable) and (right_token.str[:4] in ['M_PI', '3.14']):
                    self.add_cf_constraint(left_token, left_name, [{'radian': 1.0}], con.CF_3)
                elif (right_token.variable) and (left_token.str[:4] in ['M_PI', '3.14']):
                    self.add_cf_constraint(right_token, right_name, [{'radian': 1.0}], con.CF_3)


    def collect_angle_unit_constraints_II(self, token, left_token, right_token):
        if token.isOp and token.str in ['+', '-', '+=', '-=', '=', '<', '<=', '==', '!=', '>', '>=']:
            if left_token and right_token:
                left_name = left_token.str
                right_name = right_token.str

                if left_token.str == '.' or left_token.str == '[' or left_token.str == '(':
                    (left_token, left_name) = self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(left_token)
                if right_token.str == '.' or right_token.str == '[' or right_token.str == '(':
                    (right_token, right_name) = \
                            self.my_symbol_helper.find_compound_variable_and_name_for_dot_operand(right_token)

                if (token.str in ['+=', '-=', '=']) and self.current_child_vars:
                    if left_token.variable:
                        self.add_cf_constraint(left_token, left_name, [{'degree_360': 1.0}], con.CF_2)
                    self.current_child_vars = []

                if ({'radian': 1.0} in left_token.units) and right_token.isNumber and (abs(float(self.clean_float_string(right_token.str))) > 6.3):
                    # number approx greater than 2*pi value
                    if left_token.variable:
                        self.add_cf_constraint(left_token, left_name, [{'degree_360': 1.0}], con.CF_2) 
                    self.current_child_vars.append((left_token, left_name))
                elif ({'radian': 1.0} in right_token.units) and left_token.isNumber and (abs(float(self.clean_float_string(left_token.str))) > 6.3):
                    # number approx greater than 2*pi value
                    if right_token.variable:
                        self.add_cf_constraint(right_token, right_name, [{'degree_360': 1.0}], con.CF_2)
                    self.current_child_vars.append((right_token, right_name))
                else:
                    self.current_child_vars = []
        elif token.isOp:
            self.current_child_vars = []


    def collect_naming_constraints(self, token, left_token, right_token):
        if token.variable and (not token.units):
            #var_name = token.str
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            
            if len(var_name) < self.MIN_VAR_NAME_LENGTH:
                return
            if not token:
                return
            if token.units:
                return

            if not (self.my_symbol_helper.should_have_unit(token, var_name)):
                return
                        
            #TODO should we store variable object instead of token?
            var = con.get_variable_id(token, var_name)
            if (var and (not con.is_nm_constraint_present(var))) or (not var):
                #print var_name, token.file, token.linenr
                estimation_dict = self.type_miner.predict_proba(var_name)
                if estimation_dict:
                    estimation_list_sorted = sorted(estimation_dict.items(), key=itemgetter(1))
                    estimation_list_sorted.reverse()
                    estimation_list_sorted = map(lambda (u, p): (eval(u), p), estimation_list_sorted)

                    est_list = estimation_list_sorted[:3]
                    i = 0
                    for (u, p) in est_list:
                        #(u, p) = estimation_list_sorted[0]
                        if u in [{'meter': 1.0, 'second': -1.0}, {'meter': 1.0, 'second': -2.0}]:
                            if any(substr in var_name.lower() for substr in ['th', 'ang', 'rot', 'yaw', 'pan', 'tilt']):
                                if u == {'meter': 1.0, 'second': -1.0}:
                                    estimation_list_sorted[i] = ({'second': -1.0}, p)
                                elif u == {'meter': 1.0, 'second': -2.0}:
                                    estimation_list_sorted[i] = ({'second': -2.0}, p)
                        i+=1

                    con.add_nm_constraint(token, var_name, estimation_list_sorted)
                else:
                    con.add_nm_constraint(token, var_name, [({},0.0)])


    def collect_deep_network_naming_constraints(self, token, left_token, right_token):
        if token.variable and (not token.units):
            #var_name = token.str
            (token, var_name) = self.my_symbol_helper.find_compound_variable_and_name_for_variable_token(token)
            # if not token or token.units:
                # return

            # DON"T DO SHORT VARIABLES
            if len(var_name) < self.MIN_VAR_NAME_LENGTH:
                return
            # NEED A TOKEN
            if not token:
                return
            # EXIT ON STRONG UNITS - THIS MEANS WE ADD A NAMING CONSTRAINT WHEN THE UNITS ARE WEAK
            if (token.units and 
                 (not token.is_unit_propagation_based_on_constants) and
                 (not token.is_unit_propagation_based_on_unknown_variable)):
                return

            if not (self.my_symbol_helper.should_have_unit(token, var_name)):
                return
                        
            #TODO should we store variable object instead of token?
            var = con.get_variable_id(token, var_name)
            if (var and (not con.is_nm_constraint_present(var))) or (not var):
                #print var_name, token.file, token.linenr
                estimation_dict = self.vnh.predict_units_for_var_name(var_name, 'lstm_most_common')
                if estimation_dict:
                    estimation_list_sorted = sorted(estimation_dict.items(), key=itemgetter(0), reverse=True)
                    # SWAP ORDER TO MAKE LIST CONSISTET WITH PHYS
                    estimation_list_sorted = [(b,a) for (a,b) in estimation_list_sorted]

                    # print ('%s: %s' % (var_name, estimation_list_sorted))
                    # estimation_list_sorted = map(lambda (u, p): (u, p), estimation_list_sorted)
                    # estimation_list_sorted = map(lambda (u, p): (eval(u), p), estimation_list_sorted)
                    con.add_nm_constraint(token, var_name, estimation_list_sorted)
                else:
                    con.add_nm_constraint(token, var_name, [({},0.0)])


    def propagate_units_across_connectors(self, token, left_token, right_token, connector):
        left_units = right_units = []
        if left_token:
            left_units = left_token.units
        if right_token:
            right_units = right_token.units

        # APPLY SET UNION
        if token.str == connector:
            if connector in ['=', ':', ',']:
                if not (left_units and right_units):
                    if left_token and (left_token.isNumber or left_token.isDimensionless or left_token.str[:4] in ['M_PI']):
                        self.perform_union_when_empty = True
                    if right_token and (right_token.isNumber or right_token.isDimensionless or right_token.str[:4] in ['M_PI']):
                        self.perform_union_when_empty = True
            else:
                self.perform_union_when_empty = True

            new_units = self.merge_units_by_set_union(left_units, right_units)
            # UNIFY TOKENS FROM CHILDREN WITH CURRENT TOKENS
            if new_units != token.units:
                token.units = new_units
                self.was_some_unit_changed = True
            self.apply_propagation_status_to_token(token, left_token, right_token)


    def propagate_units_across_dot_connectors(self, token, left_token, right_token):
        self.propagate_units_across_connectors(token, left_token, right_token, '.')
        # DOT CONNECTOR AFTER PAREN WiTH NOTHING BEFORE
        if token and token.str == '.':
            if (left_token and left_token.isKnown) or \
                    (right_token and right_token.isKnown):
                token.isKnown = True

            if token.astParent and token.astParent.str == '(' and \
                    token.isKnown and (not token.astParent.astOperand2):
                self.propagate_units_across_connectors(token.astParent, token, None, '(')


    def propagate_units_across_double_colon(self, token, left_token, right_token):
        self.propagate_units_across_connectors(token, left_token, right_token, '::')


    def propagate_units_across_parenthesis(self, token, left_token, right_token):
        if token.str == '(':
            if left_token.function or ({'nounit': 0.0} in left_token.units):
                # IF LEFT SIDE IS A FUNCTION, PROPAGATE ITS RETURN UNITS
                if (left_token.function and (not left_token.function.maybe_generic_function)) or \
                        ({'nounit': 0.0} in left_token.units):
                    self.update_units_from_to(left_token, token)
        
                if (not token.units):
                    if left_token.function.return_arg_var_nr > 0:
                        #print left_token.function.name, left_token.function.return_arg_var_nr, left_token.function.arg_units
                        unit_list = left_token.function.arg_units[left_token.function.return_arg_var_nr - 1]
                        if unit_list:
                            units = (unit_list[-1])['units']
                            for u in units:
                                if u not in token.units:
                                    token.units.append(u)
                                    self.was_some_unit_changed = True
                    elif left_token.function.return_arg_var_nr == -1:
                        self.propagate_units_over_arg_expr(left_token.function.return_expr_root_token)
                        for u in left_token.function.return_expr_root_token.units:
                            if u not in token.units:
                                token.units.append(u)
                                self.was_some_unit_changed = True

            elif left_token.units:
                self.update_units_from_to(left_token, token)

            elif left_token.isName:
                return
            elif not right_token:
                #self.propagate_units_across_connectors(token, left_token, right_token, '(')
                pass
            else:
                pass


    def propagate_units_across_square_brackets(self, token, left_token, right_token):
        #self.propagate_units_across_connectors(token, left_token, right_token, '[')
        if token.str == '[':
            left_units = [] 
            if left_token:
                left_units = left_token.units

            if left_units != token.units:
                token.units = left_units
                self.was_some_unit_changed = True
            self.apply_propagation_status_to_token(token, left_token, None)


    def propagate_units_across_assignment(self, token, left_token, right_token):
        # PREVENT PROPAGATION ACROSS MULTIPLE INITIALIZATIONS, LIKE
        # x = y = z = 0 TODO Why?
        if token.str == "=":
            self.apply_propagation_status_to_token(token, left_token, right_token)
            if token.isRoot:
                return
            if left_token and left_token.str == "=":
                return
            if right_token and right_token.str == "=":
                return
        self.propagate_units_across_connectors(token, left_token, right_token, '=')


    def propagate_units_across_return(self, token, left_token, right_token):
        if token.str == 'return':
            if (not token.units) and token.astOperand1.variable:
                if token.astOperand1.variable.isArgument and token.astOperand1.scope.type == 'Function':
                    fun = token.astOperand1.scope.function
                    #print fun.name
                    #print fun.argumentId
                    #print fun.argument
                    for argnr, arg in fun.argument.items():
                        if arg == token.astOperand1.variable:
                            break
                    token.astOperand1.scope.function.return_arg_var_nr = eval(argnr)
                    #print argnr
            elif (not token.units) and (not self.found_non_arg_var) and self.found_arg_var:
                if token.astOperand1.scope.type == 'Function':
                    token.astOperand1.scope.function.return_arg_var_nr = -1
                    token.astOperand1.scope.function.return_expr_root_token = token.astOperand1

            if token.scope.function and (not token.scope.function.return_arg_var_nr):
                self.propagate_units_across_connectors(token, left_token, right_token, 'return')
                token.scope.function.is_unit_propagation_based_on_constants = \
                        token.is_unit_propagation_based_on_constants
                token.scope.function.is_unit_propagation_based_on_unknown_variable = \
                        token.is_unit_propagation_based_on_unknown_variable
                    

    def propagate_units_sqrt(self, token, left_token, right_token):
        if token.str == 'sqrt':
            unit_receiver = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    unit_receiver = token.astParent

            if not unit_receiver:
                return

            left_units = right_units = None
            if unit_receiver.astOperand1:
                left_units = unit_receiver.astOperand1.units
            if unit_receiver.astOperand2:
                right_units = unit_receiver.astOperand2.units
            # GET UNITS FROM INSIDE PARENS
            self.perform_union_when_empty = True
            new_units = self.merge_units_by_set_union(left_units, right_units)

            # DIVIDE UNITS BY TWO
            for u in new_units:
                if u in self.my_symbol_helper.dimensionless_units:
                    continue
                for k, v in u.iteritems():
                    u[k] = v / 2.

            # ATTEMPT TO PROPAGATE UNITS ACROSS '('
            for u in new_units:
                if u not in unit_receiver.units:
                    unit_receiver.units.append(u)
                    self.was_some_unit_changed = True
            self.apply_propagation_status_to_token(unit_receiver, unit_receiver.astOperand1, unit_receiver.astOperand2)
                    

    def propagate_units_inverse_trig(self, token, left_token, right_token):
        if token.str in ['atan2', 'acos', 'asin', 'atan']:
            unit_receiver = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                unit_receiver = token.astParent.astParent
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                unit_receiver = token.astParent

            if not unit_receiver:
                return

            self.update_units_from_to(token, unit_receiver)


    def propagate_units_getXYZ(self, token, left_token, right_token):
        if token.str in ['getX', 'getY', 'getZ']:
            unit_receiver = None
            if token.astParent.str == '(':
                unit_receiver = token.astParent
            
            if not unit_receiver:
                return

            is_updated = self.update_units_from_to(token, unit_receiver)
            if is_updated:
                # THIS IS TRUE BECAUSE getX is a weaker inference
                # - usually meters but might be m/s
                self.is_unit_propagation_based_on_weak_inference = True
                token.is_unit_propagation_based_on_weak_inference = True
                    

    def propagate_units_ternary(self, token, left_token, right_token):
        ''' TERNARY OPERATOR    x = 1 > 0 ? true_part : false_part
            '''
        if token.str == '?':
            if not right_token or not right_token.str == ':':
                return
            self.propagate_units_across_connectors(right_token,
                                                   right_token.astOperand1,
                                                   right_token.astOperand2,
                                                   ':')
            if right_token.units:
                self.update_units_from_to(right_token, token)
                        

    def propagate_units_pow(self, token, left_token, right_token):
        if token.str == 'pow':
            unit_receiver = None
            comma_token = None
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    if unit_receiver.astOperand2 and unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not comma_token or not unit_receiver:
                return

            if comma_token.astOperand2 and comma_token.astOperand2.isNumber:
                s = comma_token.astOperand2.str.replace('f', '')
                power_exponent = float(s)
                if comma_token.astOperand1.units:
                    # APPLY POWER TO UNITS
                    new_units = []
                    # FIRST, DEEPCOPY
                    new_units = copy.deepcopy(comma_token.astOperand1.units)
                    # NEXT, APPLY EXPONENET
                    for unit_dict in new_units:
                        if unit_dict in self.my_symbol_helper.dimensionless_units:
                            continue
                        for k, v in unit_dict.iteritems():
                            unit_dict[k] = power_exponent * v

                    for u in new_units:
                        if u not in unit_receiver.units:
                            unit_receiver.units.append(u)
                            self.was_some_unit_changed = True
                    self.apply_propagation_status_to_token(unit_receiver, comma_token.astOperand1, None)
                            

    def propagate_units_math_fmod_fmodf_fmodl(self, token, left_token, right_token):
        if token.str in ['fmod', 'fmodf', 'fmodl']:
            unit_receiver = None
            comma_token = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    if unit_receiver.astOperand2 and unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2 and token.astParent.astOperand2.str == ',':
                    comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not unit_receiver or not comma_token:
                return

            left = right = None
            if comma_token.astOperand1:
                left = comma_token.astOperand1
            if comma_token.astOperand2:
                right = comma_token.astOperand2
            # SIDE EFFECT OF NEXT LINE SHOULD ADD UNITS TO '('
            self.propagate_units_across_connectors(comma_token, left, right, ',')
            # ATTEMPT TO PROPAGATE UNITS ACROSS '('
            self.update_units_from_to(comma_token, unit_receiver)


    def propagate_units_math_min_max(self, token, left_token, right_token):
        if token.str in ['min', 'max']:
            unit_receiver = None
            comma_token = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    if unit_receiver.astOperand2 and unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2 and token.astParent.astOperand2.str == ',':
                    comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not unit_receiver or not comma_token:
                return

            left = right = None
            if comma_token.astOperand1:
                left = comma_token.astOperand1
            if comma_token.astOperand2:
                right = comma_token.astOperand2
            # SIDE EFFECT OF NEXT LINE SHOULD ADD UNITS TO '('
            self.propagate_units_across_connectors(comma_token, left, right, ',')
            # ATTEMPT TO PROPAGATE UNITS ACROSS '('
            self.update_units_from_to(comma_token, unit_receiver)
                    

    def propagate_units_math_abs_fabs_floor_ceil(self, token, left_token, right_token):
        if token.str in ['abs', 'fabs', 'floor', 'ceil']:
            unit_receiver = None
            right_token = None
            # CASE: NAMESPACE
            if token.astParent.str == '::' and token.astParent.astParent:
                unit_receiver = token.astParent.astParent
                if unit_receiver.astOperand2:
                    right_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    right_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not right_token or not unit_receiver:
                return

            # ATTEMPT TO PROPATE UNITS ACROSS '('
            self.update_units_from_to(right_token, unit_receiver)
                    

    def propagate_units_across_operators(self, token, left_token, right_token):
        '''  PROPAGATE UNITS ACROSS OPERATORS
            input:  AN AST TOKEN
            returns: None
        '''
        if token.str not in ['+', '-', '+=', '-=', '*', '/', '*=', '/=']:
            return
        if not token.isOp:
            return

        left_units = right_units = []
        if left_token:
            left_units = left_token.units
        if right_token:
            right_units = right_token.units

        new_units = []
        do_propagate = True

        # PROPAGATE EVEN IF WE DON'T KNOW -- ASSUME SCALAR
        if not (left_units and right_units):
            if not right_token:
                # HANDLE UNARY OPERATOR
                self.perform_union_when_empty = True
            if left_token and (left_token.isNumber or left_token.isDimensionless or left_token.str[:4] in ['M_PI']):
                self.perform_union_when_empty = True
            if right_token and (right_token.isNumber or right_token.isDimensionless or right_token.str[:4] in ['M_PI']):
                self.perform_union_when_empty = True

            new_units = self.merge_units_by_set_union(left_units, right_units)

            if new_units == [{'nounit': 0.0}]:
                new_units = []
 
            # CHECK FOR DIVISION AND EMPTY LEFT BRANCH
            if token.str in ['/', '/='] and not left_units:
                # FLIP SIGNS ON NEW UNITS
                for u in new_units:
                    for k, v in u.iteritems():
                        u[k] = -1 * v
            # WEAKEN INFERENCE IF WE'RE MULTIPLYING OR
            # DIVIDING ON CONSTANTS OR UNKNOWN VARIABLES
            if token.str in ['*', '/', '*=', '/=']:
                #if new_units in self.my_symbol_helper.dimensionless_units_as_lists:
                #    new_units = []

                # DETECT OPERATIONS ON CONSTANTS, like x = y * 1024.23
                # DETECT OPERATIONS ON UNKNOWN VARIABLES
                if left_token and not left_units and (not left_token.isDimensionless): #and right_units:
                    if left_token.isNumber:  # TRUE FOR EITHER INT OR FLOAT
                        self.is_unit_propagation_based_on_constants = True
                        token.is_unit_propagation_based_on_constants = True
                    elif left_token.str[:4] not in ['M_PI']:
                        self.is_unit_propagation_based_on_unknown_variable = True
                        token.is_unit_propagation_based_on_unknown_variable = True
                if right_token and not right_units and (not right_token.isDimensionless): #and left_units:
                    if right_token.isNumber:  # TRUE FOR EITHER INT OR FLOAT
                        self.is_unit_propagation_based_on_constants = True
                        token.is_unit_propagation_based_on_constants = True
                    elif right_token.str[:4] not in ['M_PI']:
                        self.is_unit_propagation_based_on_unknown_variable = True
                        token.is_unit_propagation_based_on_unknown_variable = True

                do_propagate = False

        else:
            # ADDITION / SUBTRACTION - ATTEMPT TO MERGE
            if token.str in ['+', '-', '+=', '-=']:
                new_units = self.merge_units_by_set_union(left_units, right_units)
            # MULTIPLICATION / DIVISION
            elif token.str in ['*', '*=', '/', '/=']:
                all_unit_dicts_from_multiplication = []
                for unit_dict_left in left_units:
                    for unit_dict_right in right_units:
                        result_units = self.apply_multiplication_to_unit_dicts(
                                unit_dict_left,
                                unit_dict_right,
                                token.str)
                        if result_units:
                            all_unit_dicts_from_multiplication.append(result_units)
                for u in all_unit_dicts_from_multiplication:
                    if u not in new_units:
                        new_units.append(u)

                if new_units == []:
                    token.isDimensionless = True
                elif {'wrong': 0.0} in new_units:
                    new_units = []

        # UNIFY TOKENS FROM CHILDREN WITH CURRENT TOKEN
        if new_units != token.units:
            #if token.units and (not new_units):
            #    return
            token.units = new_units
            self.was_some_unit_changed = True
        if do_propagate:
            self.apply_propagation_status_to_token(token, left_token, right_token)
      

    def apply_multiplication_to_unit_dicts(self, unit_dict_left, unit_dict_right, op):
        ''' APPLIES MULTIPLICATION AND DIVISION TO UNIT DICTIONARIES
            (BY ADDING EXPONENTS)
            input:  unit_dict_left   dictionary of units, eg:  {'m':1, 's':-1} 
                    unit_dict_right  same
                    op   string representing mult or div operators
            returns: new dict  with resulting units  eg: {'m':2, 's':-2}
            '''
        
        #if unit_dict_left == {'radian': 1.0} and unit_dict_right == {'degree_360_unit': 1.0}:
        #    return {'degree_360': 1.0}
        #elif unit_dict_left == {'degree_360_unit': 1.0} and unit_dict_right == {'radian': 1.0}:
        #    return {'degree_360': 1.0}
        #elif unit_dict_left == {'degree_360': 1.0} and unit_dict_right == {'radian_unit': 1.0}:
        #    return {'radian': 1.0}
        #elif unit_dict_left == {'radian_unit': 1.0} and unit_dict_right == {'degree_360': 1.0}:
        #    return {'radian': 1.0}

        if unit_dict_right == {'degree_360_unit': 1.0}:
            if unit_dict_left == {'radian': 1.0}:
                return {'degree_360': 1.0}
            elif unit_dict_left == {'second': -1.0}:
                return {'degree_360': 1.0, 'second': -1.0}
            else:
                return {'wrong': 0.0}
        elif unit_dict_left == {'degree_360_unit': 1.0}:
            if unit_dict_right == {'radian': 1.0}:
                return {'degree_360': 1.0}
            elif unit_dict_right == {'second': -1.0}:
                return {'degree_360': 1.0, 'second': -1.0}
            else:
                return {'wrong': 0.0}
        elif unit_dict_right == {'radian_unit': 1.0}:
            if unit_dict_left == {'degree_360': 1.0}:
                return {'radian': 1.0}
            elif unit_dict_left == {'degree_360': 1.0, 'second': -1.0}:
                return {'second': -1.0}
            else:
                return {'wrong': 0.0}
        elif unit_dict_left == {'radian_unit': 1.0}:
            if unit_dict_right == {'degree_360': 1.0}:
                return {'radian': 1.0}
            elif unit_dict_right == {'degree_360': 1.0, 'second': -1.0}:
                return {'second': -1.0}
            else:
                return {'wrong': 0.0}

        return_dict = {}

        # SPECIAL HANDLING FOR RADIANS AND QUATERNIONS
        if unit_dict_left in self.my_symbol_helper.dimensionless_units \
                and unit_dict_right in self.my_symbol_helper.dimensionless_units:
            # SPECIAL CASE BOTH ARE RADIANS.  CLOSED UNDER MULTIPLICATION
            if op in ['*', '*=']:
                return copy.deepcopy(unit_dict_left)
        elif unit_dict_left in self.my_symbol_helper.dimensionless_units:
            # DON'T PROPAGATE RADIANS
            unit_dict_left = {}
        elif unit_dict_right in self.my_symbol_helper.dimensionless_units:
            # DON'T PROPAGATE RADIANS
            unit_dict_right = {}

        return_dict = copy.deepcopy(unit_dict_left)
        for unit in unit_dict_right:
            if unit in return_dict:
                # BOTH DICTS HAVE SAME UNIT
                if op in ['*', '*=']:
                    # ADD OF EXPONENT IS MULT
                    return_dict[unit] += unit_dict_right[unit]
                elif op in ['/', '/=']:
                    # SUBTRACTION OF EXPONENT IS DIV
                    return_dict[unit] -= unit_dict_right[unit]
            else:
                # ONLY ONE SIDE HAS UNIT
                if op in ['*', '*=']:
                    # COPY - THIS IS NOT A REFERNCE
                    return_dict[unit] = unit_dict_right[unit]
                elif op in ['/', '/=']:
                    return_dict[unit] = -1 * unit_dict_right[unit]

        # FILTER OUT ZEROs - UNITLESS
        return_dict = {k: v for k, v in return_dict.items() if v != 0}
        return return_dict
      

    def merge_units_by_set_union(self, left_units, right_units):
        ''' input: {left, right}_units - lists of unit dictionaries.
            result: set union of inputs
            '''
        if self.perform_intersection:
            return self.merge_units_by_set_intersection(left_units, right_units)

        new_units = []
                
        if left_units and right_units:
            if left_units == right_units:
                # COPY EITHER ONE BECAUSE SAME
                new_units = copy.deepcopy(left_units)
            else:
                new_units = copy.deepcopy(left_units)
                for r in right_units:
                    if r not in new_units:
                        new_units.append(copy.deepcopy(r))       
        else:
            if left_units:
                new_units = copy.deepcopy(left_units)
            elif right_units:
                new_units = copy.deepcopy(right_units)

        return new_units


    def merge_units_by_set_intersection(self, left_units, right_units):
        ''' input: {left, right}_units - lists of unit dictionaries.
            result: set intersection of inputs
            '''
        new_units = []

        if self.perform_union_when_empty:
            if not (left_units and right_units):
                if left_units:
                    new_units = copy.deepcopy(left_units)
                elif right_units:
                    new_units = copy.deepcopy(right_units)
                self.perform_union_when_empty = False
                return new_units
                                 
        for r in right_units:
            if r in left_units:
                new_units.append(copy.deepcopy(r))

        self.perform_union_when_empty = False        
        return new_units


    def update_units_from_to(self, from_token, to_token):
        is_updated = False
        for u in from_token.units:
            if u not in to_token.units:
                to_token.units.append(u)
                self.was_some_unit_changed = True
                is_updated = True

        self.apply_propagation_status_to_token(to_token, from_token, None)

        return is_updated


    def apply_propagation_status_to_token(self, token, left_token, right_token):
        ''' APPLIES PROPAGATION WEAKENING FROM CHILD TOKENS TO PARENT TOKEN
            input: token, left_token, right_token
            returns: none.  side effect can change variables on 'token'
            '''
        left_constants = False
        left_unknown_variable = False
        right_constants = False
        right_unknown_variable = False

        if left_token:
            if left_token.isKnown:
                token.isKnown = True
            if left_token.is_unit_propagation_based_on_weak_inference:
                token.is_unit_propagation_based_on_weak_inference = True

            left_constants = left_token.is_unit_propagation_based_on_constants
            left_unknown_variable = left_token.is_unit_propagation_based_on_unknown_variable

        if right_token:
            if right_token.isKnown:
                token.isKnown = True
            if right_token.is_unit_propagation_based_on_weak_inference:
                token.is_unit_propagation_based_on_weak_inference = True

            right_constants = right_token.is_unit_propagation_based_on_constants
            right_unknown_variable = right_token.is_unit_propagation_based_on_unknown_variable

        token.is_unit_propagation_based_on_constants = left_constants or right_constants
        token.is_unit_propagation_based_on_unknown_variable = left_unknown_variable or right_unknown_variable
         

    def apply_known_status_to_token(self, token, left_token, right_token, both=False):
        ''' APPLIES STATUS FROM CHILD TOKENS TO PARENT TOKEN
            input: token, left_token, right_token
            returns: none.  side effect can change variables on 'token'
            '''
        left_known = False
        right_known = False

        if left_token:
            left_known = left_token.isKnown

        if right_token:
            right_known = right_token.isKnown

        if both:
            token.isKnown = left_known and right_known
        else:
            token.isKnown = left_known or right_known


    def find_first_variable_token(self, token, left_token, right_token):
        ''' ASSUME THIS WILL BE CALLED ON LHS OF ASSIGNMENT.
            SHOULD ONLY BE ONE VARIABLE
            '''
        if token.variable:
            self.return_value_list.append(token)

    def recurse_and_collect_string(self, token):
        ''' SIMPLE RECURSION FOR LEFT-HAND SIDE OF ASSIGNMENT STATMENTS
            '''
        # PROTECT FROM NULL
        if not token:
            return ''
        my_return_string = ''
        # LEFT RECURSE
        if token.astOperand1:
            my_return_string +=  self.recurse_and_collect_string(token.astOperand1)
        # SELF
        my_return_string += token.str
        # RIGHT RECURSE
        if token.astOperand2:
            my_return_string += self.recurse_and_collect_string(token.astOperand2)
        if token.str == '[':
            my_return_string += ']'
        return my_return_string


    def collect_function_param_units_and_decorate_function(self, token, left_token, right_token):
        ''' COLLECT AVAILABLE UNITS ON FUNCTION PARAMETERS FROM AST 
            AND ADD THEM TO FUNCTION OBJECT
            '''
        if token.function:
            function_args_units = []
            if token.astParent.astOperand2:
                if token.astParent.astOperand2.str == ',':
                    # MULITPLE ARGS
                    function_args_units = self.recurse_on_function_args(token.astParent.astOperand2)
                else:
                    # ONLY ONE ARG
                    if token.astParent.astOperand2.units:                        
                        function_args_units = [(token.astParent.astOperand2, token.astParent.astOperand2.units)]
            if function_args_units:
                if len(function_args_units) != len(token.function.arg_units):
                    return
                for i, u in enumerate(function_args_units):
                    (t, un) = u
                    new_dict = {'linenr': int(token.linenr),
                                'units': un,
                                'token': t,
                                'function': token}
                    if new_dict not in token.function.arg_units[i]:
                        token.function.arg_units[i].append(new_dict)


    def recurse_on_function_args(self, comma_token):
        ''' RECURSIVELY COLLECT UNITS FOR FUNCTION ARGUMENTS
            input: ccpcheck token object - str=','
            output: list of units in arg order
            '''
        my_return_list = []
        if comma_token.astOperand1:
            if comma_token.astOperand1.str == ',':
                left_branch_units_list = self.recurse_on_function_args(comma_token.astOperand1)
                if left_branch_units_list:
                    for u in left_branch_units_list:
                        my_return_list.append(u)
            else:
                my_return_list.append((comma_token.astOperand1, comma_token.astOperand1.units))
        if comma_token.astOperand2:
            my_return_list.append((comma_token.astOperand2, comma_token.astOperand2.units))
        return my_return_list


    def clean_float_string(self, float_string):
        ''' REMOVES TRAILING 'f' OR 'l' FROM NUMBER, like '180.0f' becomes '180.0'
            input: string
            output string
            '''
        float_string = float_string.lower()
        float_string = float_string.replace('f','')
        float_string = float_string.replace('l','')
        float_string = float_string.replace('u','')
        return float_string


    def apply_latest_arg_units(self, token, left_token, right_token):
        if token.variable and token.variable.isArgument:
            fun = token.scope.function

            for argnr, arg in fun.argument.items():
                if arg == token.variable:
                    break

            arg_var_nr = eval(argnr)
            unit_list = fun.arg_units[arg_var_nr - 1]
            if unit_list:
                units = (unit_list[-1])['units']
                for u in units:
                    if u not in token.units:
                        token.units.append(u)
                        self.was_some_unit_changed = True
                        self.found_units_in_this_tree = True


    def propagate_units_over_arg_expr(self, root_token):
        tw = TreeWalker(None)  

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0

        tw.is_unit_propagation_based_on_constants = False
        tw.is_unit_propagation_based_on_unknown_variable = False

        # RESET TOKENS
        tw.generic_recurse_and_apply_function(root_token, tw.reset_tokens)

        # RESET THE TREE WALKER'S LINE NUMBERS
        tw.reset_min_max_line_numbers()
        # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
        tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)


        # APPLY ARG UNITS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_latest_arg_units)       
        # APPLY UNITS TO KNOWN SYMBOLS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_known_symbols_units)
        # APPLY UNITS TO CONVERSION FACTORS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_conversion_factor_units)
        # APPLY DIMENSIONLESS UNITS
        tw.generic_recurse_and_apply_function(root_token, tw.apply_dimensionless_units)
            

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


